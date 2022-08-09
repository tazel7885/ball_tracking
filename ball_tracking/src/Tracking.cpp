
#include <ball_tracking/Tracking.h>

using namespace std;

Tracking::Tracking(ros::NodeHandle nh) 
    : nh(nh),
      topic_pos("/ball_pos"),
      topic_ball("/detected_ball")
{
    Initialize();
    Process();
}

Tracking::~Tracking(){}

void Tracking::Initialize()
{
    dt = 0.05;
    count = 0;
    
    is_initialized = false;
    detect_ball = false;
    detected_ball = false;
    check = false;

    filtering_ball.x = 0.0;
    filtering_ball.y = 0.0;

    ball_vel[2] = {0, };

    ball_direction[2] = {0, };

    // create a 4D state vector, [x, vx, ax, y, vy, ay]T
    kf.x = Eigen::VectorXd(6);

    // measurement matrix
    kf.H = Eigen::MatrixXd(2,6);
    kf.H << 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0;

    // measurement covariance
    kf.R = Eigen::MatrixXd(2,2);
    kf.R << 0.1, 0, //0.1
            0, 0.1;
        
    // the initial transition matrix F_
    kf.F = Eigen::MatrixXd(6,6);
    kf.F << 1, dt, dt*dt/2, 0,  0,       0,
            0, 1,       dt, 0,  0,       0,
            0, 0,        1, 0,  0,       0,
            0, 0,        0, 1, dt, dt*dt/2,
            0, 0,        0, 0,  1,      dt,
            0, 0,        0, 0,  0,       1;

    // set the process covariance matrix Q
    kf.Q = Eigen::MatrixXd(6,6);
    kf.Q << 0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0.01;

    sub_depth = nh.subscribe(topic_pos, 1, &Tracking::BallDepthCallback, this);
    sub_detect_ball = nh.subscribe(topic_ball, 1, &Tracking::DetectedBallCallback, this);

    pub_ball_pos = nh.advertise<geometry_msgs::Pose2D>("/kf_ball_pos", 10);
    pub_ball_vel = nh.advertise<geometry_msgs::Pose2D>("/kf_ball_vel", 10);
    pub_predict_ball_pos = nh.advertise<geometry_msgs::Pose2D>("/predict_ball_pos", 10);

    nh.getParam("/gui", gui);
    gui = "ON";
    if(gui == "ON")
    {
		color_image_map_path = ros::package::getPath("ball_tracking") + "/map/test_field.jpg";
		color_map_image = cv::imread(color_image_map_path.c_str());

		#if (CV_MAJOR_VERSION <= 3) 
		{
			// for opencv 3.4.0 user
			//cv::namedWindow("ball tracking image", CV_WINDOW_NORMAL);
			//if(mode == "on_alice")
			//{
			//	cv::cvSetWindowProperty("ball tracking image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			//}
		}
		#else
		{
			// for opencv 4.1.1 user
			//cv::namedWindow("ball tracking image", CV_RAND_NORMAL);
		}
		#endif
	}

}

void Tracking::BallDepthCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    sensing_ball.x = msg->x;
    sensing_ball.y = msg->y;

    check = true;
}

void Tracking::DetectedBallCallback(const std_msgs::Bool::ConstPtr& msg)
{
    detect_ball = msg->data;
}

void Tracking::Process()
{
    ros::Rate loop_rate(1/dt);

    while(ros::ok())
    {
        if(check)
        {   
            UpdateBallPosition();
            Filtering();
            BallPredictPos();
            BallPub();
            
            if(gui == "ON")
            {
                Visualize();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return ;
}

void Tracking::UpdateBallPosition()
{
    update_ball = sensing_ball;
    detected_ball = detect_ball;
}
 
void Tracking::Filtering()
{
    if(!is_initialized && detected_ball)
    {
        kf.x << update_ball.x, 0, 0,
                update_ball.y, 0, 0;

        kf.P = Eigen::MatrixXd(6,6);
        
        kf.P << 1, 0, 0, 0, 0, 0,  
                0, 10, 0, 0, 0, 0,
                0, 0, 100, 0, 0, 0,
                0, 0, 0, 1, 0, 0,   
                0, 0, 0, 0, 10, 0,
                0, 0, 0, 0, 0, 100;

        is_initialized = true;

        return;
    }

    else if(detected_ball)
    {
        Eigen::VectorXd raw_ball = Eigen::VectorXd(2);
        raw_ball << update_ball.x, update_ball.y;

        kf.Predict();
        kf.Update(raw_ball);

        filtering_ball.x = kf.x(0);
        filtering_ball.y = kf.x(3);

        ball_speed.x = kf.x(1);
        ball_speed.y = kf.x(4);

        return;
    }

    else if(is_initialized && !detected_ball)
    {
        kf.Predict();

        if(count > 20)
        {
            count = 0;
            is_initialized = false;

            return;
        }

        filtering_ball.x = kf.x(0);
        filtering_ball.y = kf.x(3);

        ball_speed.x = kf.x(1);
        ball_speed.y = kf.x(4);

        count++;
    }
}

void Tracking::BallPredictPos()
{
    kf_pred = kf;
    float predict_time = 1.0;
    for(int i=0; i< (predict_time / dt); i++) {kf_pred.Predict();}
    cout << predict_time / dt << endl;
    cout << dt << endl;
    cout << predict_time << endl;
    predict_ball.x = kf_pred.x(0);
    predict_ball.y = kf_pred.x(3);
}

void Tracking::BallPub()
{
    geometry_msgs::Pose2D ball_velocity;
    geometry_msgs::Pose2D ball_position;
    geometry_msgs::Pose2D predict_ball_position;


    ball_position.x = filtering_ball.x;   // ball_kalman Pub
    ball_position.y = filtering_ball.y;

    ball_velocity.x = ball_speed.x;   // ball_speed Pub
    ball_velocity.y = ball_speed.y;

    predict_ball_position.x = predict_ball.x;   // ball_predict Pub
    predict_ball_position.y = predict_ball.y;

    pub_ball_vel.publish(ball_velocity);
    pub_ball_pos.publish(ball_position);
    pub_predict_ball_pos.publish(predict_ball_position);
}

void Tracking::Visualize()
{
    cv::Mat ball_tracking_img = color_map_image.clone();

    cv::Point2f ball_point = cv::Point2f((filtering_ball.x) * 100 + ball_tracking_img.size().width / 2, ball_tracking_img.size().height/2 - filtering_ball.y * 100);
    if(filtering_ball.detect)
        cv::circle(ball_tracking_img, ball_point, 12, cv::Scalar(0, 255, 0), -1);  
    else
        cv::circle(ball_tracking_img, ball_point, 14, cv::Scalar(0, 0, 255), -1);  

    cv::Point2f ball_pred_point = cv::Point2f((predict_ball.x) * 100 + ball_tracking_img.size().width / 2, ball_tracking_img.size().height/2 - predict_ball.y * 100);
    cv::circle(ball_tracking_img, ball_pred_point, 12, cv::Scalar(255, 255, 0), -1); 

    cv::imshow("ball tracking image", ball_tracking_img);
    cv::waitKey(1);
}
