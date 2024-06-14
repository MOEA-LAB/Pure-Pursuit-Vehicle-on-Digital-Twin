#ifndef PURE_PURSUIT__
#define PURE_PURSUIT__

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Imu.h>

#include "arc_length_spline.h"
#include "track.h"

#include <thread>
#include <chrono>

#include "serial_ros/PoseMsgs.h"
struct OutPut {
    double X;
    double Y;
    double V;
    double W;
    double R;
    double dDelta;

    void setZero()
    {
        X = 0.0f;
        Y = 0.0f;
        W = 0.0f;
        R = 0.0f;
        dDelta = 0.0f;
    }
};

class pure_pursuit
{
public:
    pure_pursuit(ros::NodeHandle &n);
    ~pure_pursuit();

private:
    int seq;

    ros::NodeHandle &nh;

    ros::Subscriber vrpn_sub;
    ros::Subscriber vrpn_sub_all;
    ros::Subscriber collision_avoid_ctrl;
    
    ros::Subscriber ekf_state_sub;
    ros::Subscriber ctrl_run_sub;
    ros::Subscriber ctrl_speed_pub;
    ros::Subscriber ctrl_switch_mode_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;
    
    void vrpnCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void vrpnCallback_all(const serial_ros::PoseMsgs &msg);
    void collision_avoid_ctrl_Callback(const std_msgs::UInt32::ConstPtr &msg);

    void ekfStateCallback(const nav_msgs::OdometryConstPtr &msg);
    void ctrl_run_callback(const std_msgs::Bool::ConstPtr &msg);
    void ctrl_speed_callback(const std_msgs::UInt32::ConstPtr &msg);
    void ctrl_switch_mode_callback(const std_msgs::UInt32::ConstPtr &msg);
    void _ImuCallback(const sensor_msgs::Imu &msg);
    void _OdomCallback(const nav_msgs::Odometry &msg);

    ros::Publisher control_pub;
    
    std::thread controlPubThread_;
    void _controlPub();

    float baseAngle = -10;
    float imu[5];
    float odom[5];
    float m_x;
    float m_y;
    float m_yaw;
    float deta_u0=0;

    std::string m_nodeName;
    std::string m_vehicleName;
    std::string m_routeFileName;
    float m_angleError;
    // 控制参数
    bool m_isRun;
    unsigned int m_speed;
    unsigned int m_mode;

    double lhd;
    double currenV;

    double cmd_vel_linear;
    double cmd_vel_angular;

    ArcLengthSpline track_;

    OutPut calcPurePursuit(const State& state,Eigen::Vector2d targetPos);

    void loadRoute(std::string filePath);
};

#endif