#include <ros/ros.h>
#include <ball_tracking/Tracking.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_tracking");
    ros::NodeHandle nh;
    Tracking tracking(nh);
    return 0;
}