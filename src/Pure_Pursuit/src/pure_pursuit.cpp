#include "pure_pursuit.h"

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>

#include <nlohmann/json.hpp>

#define PI 3.1415926
Eigen::Vector2d targetPos;
pure_pursuit::pure_pursuit(ros::NodeHandle &n) : nh(n)
{
// m_nodeName = ros::this_node::getName();
n.getParam("comm_hub/vehicle/name", m_vehicleName);
n.getParam("comm_hub/vehicle/debug", m_isRun);
n.getParam("comm_hub/vehicle/speed", currenV);
n.getParam("comm_hub/vehicle/route", m_routeFileName);
n.getParam("comm_hub/vehicle/angle_error", m_angleError);

// 生成本车ID
m_vehicleId = (m_vehicleName[m_vehicleName.size() - 1] - '0' + (m_vehicleName[m_vehicleName.size() - 2] - '0') * 10 - 70);

// 订阅全局位置信息
vrpn_sub_all = n.subscribe("/motion_capture_pose_all", 10, &pure_pursuit::vrpnCallback_all, this);

loadRoute(m_routeFileName); // 调取路径

lhd = 0.333 * currenV + 0.3; //Leading Hold Distance 计算并设置前视距离（Leading Hold Distance），计算公式为 0.333 * currenV + 0.3。
printf("lhd: %f\n", lhd);

// 订阅节点
vrpn_sub = n.subscribe("/motion_capture_pose", 10, &pure_pursuit::vrpnCallback, this);
// 订阅传感器ekf融合数据
ctrl_run_sub = n.subscribe("/ctrl_run", 10, &pure_pursuit::ctrl_run_callback, this);
ctrl_speed_pub = n.subscribe("/ctrl_speed", 10, &pure_pursuit::ctrl_speed_callback, this);
ctrl_switch_mode_pub = n.subscribe("/ctrl_switch_mode", 10, &pure_pursuit::ctrl_switch_mode_callback, this);

imu_sub = n.subscribe("/imu", 10, &pure_pursuit::_ImuCallback, this); // 用于获取车辆的姿态（包括滚转角、俯仰角和偏航角），主要用于确定车辆的方向和角速度。
odom_sub = n.subscribe("/odom", 10, &pure_pursuit::_OdomCallback, this); // 用于获取车辆的位置信息和运动状态，主要用于确定车辆的位置和线速度。

// 发布控制节点
control_pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10, true);

controlPubThread_ = std::thread(&pure_pursuit::_controlPub, this);
}

pure_pursuit::~pure_pursuit()
{
}

// 全局位置的回调函数（可加入避障）
void pure_pursuit::vrpnCallback_all(const serial_ros::PoseMsgs &msg)
{
// 获取本车位置
double Pose_x = msg.poseMsgs[m_vehicleId].pose.pose.position.x; // x坐标
double Pose_y = msg.poseMsgs[m_vehicleId].pose.pose.position.y; // y坐标
geometry_msgs::Quaternion quat = msg.poseMsgs[m_vehicleId].pose.pose.orientation;

// 计算其他车辆的位置并进行避障处理
for (size_t i = 0; i < msg.poseMsgs.size(); ++i)
{
if (i == m_vehicleId) continue; // 跳过自身

double other_x = msg.poseMsgs[i].pose.pose.position.x;
double other_y = msg.poseMsgs[i].pose.pose.position.y;
double safe_distance = 1.0;
// 检查其他车辆是否在避障范围内
double distance = sqrt(pow(other_x - Pose_x, 2) + pow(other_y - Pose_y, 2));
if (distance < safe_distance) // safe_distance为避障的安全距离
{
// 实现避障逻辑，这里可以修改目标路径或调整速度等
avoidObstacle(other_x, other_y);
}
}

// if(Pose_x<0||Pose_x>16||Pose_y<0||Pose_y>16){//当前车出界停下
//     std::cout<<"----------------\n"; 
//     std::cout<<"检测到将会超出边界"<<'\n'; 
//     std::cout<<"----------------\n"; 
//     } 
//     cmd_vel_linear=currenV; 
//     if(flag){ 
//     cmd_vel_angular=radtest; 
//     }else{ 
//     cmd_vel_angular=0; 
//     } 



}

// 自身位置的回调函数
void pure_pursuit::vrpnCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
tf::Quaternion quat;
tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
double roll, pitch, yaw;
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
yaw = yaw + PI / 2 + m_angleError;

State x0;
x0.phi = yaw;
x0.X = msg.pose.pose.position.x;
x0.Y = msg.pose.pose.position.y;

if (x0.X < 0 || x0.X > 16 || x0.Y < 0 || x0.Y > 16)
{
return;
}

if (baseAngle == -10)
{
baseAngle = x0.phi;
}

baseAngle += x0.phi - imu[0];


for (size_t i = 0; i < sizeof(imu) / sizeof(imu[0]); i++)
{
x0.phi = imu[i];
x0.X += odom[i] * cos(imu[i]) * 0.02;
x0.Y += odom[i] * sin(imu[i]) * 0.02;
}

m_x = x0.X;
m_y = x0.Y;
m_yaw = x0.phi;
}

void pure_pursuit::ekfStateCallback(const nav_msgs::OdometryConstPtr &msg)
{
tf::Quaternion quat;
tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
double roll, pitch, yaw;
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

State x0;
x0.phi = yaw + PI / 2;
x0.X = msg->pose.pose.position.x;
x0.Y = msg->pose.pose.position.y;
x0.vx = msg->twist.twist.linear.x;
x0.vy = msg->twist.twist.linear.y;
x0.r = msg->twist.twist.angular.z;



if (m_mode)
{
}
else
{
double dist = track_.porjectOnSpline(x0);
dist += lhd;
targetPos = track_.getPostion(dist);
}

if (0 > targetPos[0] || targetPos[1] < 0)
{
return;
}

// 纯跟踪计算
OutPut u0 = calcPurePursuit(x0, targetPos);

cmd_vel_linear = u0.V;
cmd_vel_angular = u0.W;
}

void pure_pursuit::_ImuCallback(const sensor_msgs::Imu &msg)
{
tf::Quaternion quat;
tf::quaternionMsgToTF(msg.orientation, quat);
double roll, pitch, yaw;
tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
yaw += baseAngle;
if (yaw < -PI)
{
yaw += 2 * PI;
}
else if (yaw > PI)
{
yaw -= 2 * PI;
}

m_yaw = yaw;


for (size_t i = 0; i < sizeof(imu) / sizeof(imu[0]) - 1; i++)
{
imu[i] = imu[i + 1];
}
imu[sizeof(imu) / sizeof(imu[0]) - 1] = yaw;
}

void pure_pursuit::_OdomCallback(const nav_msgs::Odometry &msg)
{

m_x += odom[sizeof(odom) / sizeof(odom[0]) - 1] * cos(m_yaw) * 0.02;
m_y += odom[sizeof(odom) / sizeof(odom[0]) - 1] * sin(m_yaw) * 0.02;

for (size_t i = 0; i < sizeof(odom) / sizeof(odom[0]) - 1; i++)
{
odom[i] = odom[i + 1];
}
odom[sizeof(odom) / sizeof(odom[0]) - 1] = msg.twist.twist.linear.x;
}

void pure_pursuit::ctrl_run_callback(const std_msgs::Bool::ConstPtr &msg)
{
m_isRun = msg->data;
}

void pure_pursuit::ctrl_speed_callback(const std_msgs::UInt32::ConstPtr &msg)
{
m_speed = msg->data;
}

void pure_pursuit::ctrl_switch_mode_callback(const std_msgs::UInt32::ConstPtr &msg)
{
m_mode = msg->data;
}

void pure_pursuit::_controlPub()
{
while (1)
{
State x0;
x0.phi = m_yaw;
x0.X = m_x;
x0.Y = m_y;

Eigen::Vector2d targetPos;
if (m_mode)
{
// 模式相关的逻辑
}
else
{
double dist = track_.porjectOnSpline(x0);
dist += lhd;
targetPos = track_.getPostion(dist);
}

if (0 > targetPos[0] || targetPos[1] < 0)
{
std::this_thread::sleep_for(std::chrono::milliseconds(50));
return;
}

// 纯跟踪计算
OutPut u0 = calcPurePursuit(x0, targetPos);

cmd_vel_linear = u0.V;
cmd_vel_angular = u0.W;

ros::Time curTime = ros::Time::now();
geometry_msgs::TwistStamped twist_stam;
twist_stam.header.seq = seq++;
twist_stam.header.stamp = curTime;
twist_stam.header.frame_id = "base_link";

geometry_msgs::Vector3 linear;
linear.x = cmd_vel_linear;
linear.y = 0;
linear.z = 0;
geometry_msgs::Vector3 angular;
angular.x = 0;
angular.y = 0;
angular.z = cmd_vel_angular;

if (!m_isRun)
{
linear.x = 0;
angular.z = 0;
}

twist_stam.twist.linear = linear;
twist_stam.twist.angular = angular;

control_pub.publish(twist_stam);
std::this_thread::sleep_for(std::chrono::milliseconds(20));
}
}



OutPut pure_pursuit::calcPurePursuit(const State &state, Eigen::Vector2d targetPos)
{
OutPut u0;
double ld = sqrt(pow(targetPos[0] - state.X, 2) + pow(targetPos[1] - state.Y, 2));
double sita = atan2(targetPos[1] - state.Y, targetPos[0] - state.X);
double alpha = sita - state.phi;
if (alpha < -PI)
{
alpha += 2 * PI;
}
else if (alpha > PI)
{
alpha -= 2 * PI;
}

u0.V = currenV;
u0.W = currenV * 2 * sin(alpha) / ld;

u0.X = targetPos[0];
u0.Y = targetPos[1];

return u0;
}

void pure_pursuit::loadRoute(std::string filePath)
{
ROS_INFO("loading route");
std::ifstream iTrack(filePath);
nlohmann::json jsonTrack;
iTrack >> jsonTrack;
std::vector<double> X = jsonTrack["X"];
std::vector<double> Y = jsonTrack["Y"];
Eigen::VectorXd vdX = Eigen::Map<Eigen::VectorXd>(X.data(), X.size());
Eigen::VectorXd vdY = Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size());
track_.gen2DSpline(vdX, vdY);
}

void pure_pursuit::avoidObstacle(double other_x, double other_y)
{
// 简单的避障逻辑，可以根据实际需求进行调整
// 这里假设车辆避让方向为右侧，并且避障距离为预设的距离

double avoidanceDistance = 1.0; // 避障距离，可以根据实际情况调整

// 计算新的目标点，假设避让方向为右侧
double angle_to_other = atan2(other_y - m_y, other_x - m_x);
double avoidance_angle = angle_to_other - PI / 2; // 向右90度

double newX = m_x + avoidanceDistance * cos(avoidance_angle);
double newY = m_y + avoidanceDistance * sin(avoidance_angle);

// 确保新路径点不出界
newX = std::max(0.0, std::min(newX, 16.0));
newY = std::max(0.0, std::min(newY, 16.0));

// 设置新的目标位置
targetPos = Eigen::Vector2d(newX, newY);
}

    

