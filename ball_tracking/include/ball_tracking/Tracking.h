//
// Created by tazel78885 on 2022/06/13.
//

#ifndef TRACKING_H_
#define TRACKING_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>

#include <vector>
#include <string>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ball_tracking/Kalman.h>

typedef struct
{
    float x,y;
    bool detect;
} Position;

class Tracking {
    public:
        Tracking(ros::NodeHandle nh);

        ~Tracking();

        KalmanFilter kf;
        KalmanFilter kf_clone;
        KalmanFilter kf_pred;

    private:
        ros::NodeHandle nh;
    
        void Initialize();

        void Process();
        void UpdateBallPosition();
        void Filtering();
        void BallPub();
        void Visualize();
        void BallPredictPos();
        
        void BallDepthCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
        void DetectedBallCallback(const std_msgs::Bool::ConstPtr& msg);

        ros::Subscriber sub_depth;
        ros::Subscriber sub_detect_ball;

        ros::Publisher pub_ball_pos;
        ros::Publisher pub_ball_vel;
        ros::Publisher pub_predict_ball_pos;

        std::string topic_depth;
        std::string topic_ball;
        std::string topic_pos;

        Position sensing_ball;
        Position update_ball;
        Position filtering_ball;
        Position ball_speed;
        Position predict_ball;

        bool detect_ball;
        bool detected_ball;

        int ball_direction[2];
        float ball_vel[0];

        bool is_initialized;
        float dt;

        bool check;

        int count;
        

        cv_bridge::CvImagePtr segmentation_image_ros;
        cv::Mat segmentation_image;
        std::string color_image_map_path;
        cv::Mat color_map_image;

        std::string gui;
};

#endif
