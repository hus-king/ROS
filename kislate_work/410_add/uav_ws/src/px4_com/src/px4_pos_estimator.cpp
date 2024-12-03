//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>


#include <math_utils.h>
#include <Frame_tf_utils.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nav_msgs/Odometry.h>


using namespace std;
//---------------------------------------相关参数-----------------------------------------------
bool ready_for_pub = false;
int pose_source = 0;

//---------------------------------------laser定位相关------------------------------------------
Eigen::Vector3d pos_drone_laser;                          //无人机当前位置 (laser)
Eigen::Quaterniond q_laser;
Eigen::Vector3d Euler_laser;                                         //无人机当前姿态(laser)
geometry_msgs::TransformStamped laser;                          //当前时刻cartorgrapher发布的数据
geometry_msgs::TransformStamped laser_last;

//---------------------------------------t265定位相关------------------------------------------
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d Euler_t265;
geometry_msgs::TransformStamped t265;
geometry_msgs::TransformStamped t265_last;

//---------------------------------------uwb定位相关------------------------------------------
Eigen::Vector3d pos_drone_uwb;
Eigen::Quaterniond q_uwb;
Eigen::Vector3d Euler_uwb;
geometry_msgs::TransformStamped uwb;
geometry_msgs::TransformStamped uwb_last;


double tfmini_raw;
//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;                           //无人机当前位置 (来自fcu)
Eigen::Vector3d vel_drone_fcu;                           //无人机上一时刻位置 (来自fcu)

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;                                          //无人机当前欧拉角(来自fcu)
//---------------------------------------发布相关变量--------------------------------------------
geometry_msgs::PoseStamped vision;
//geometry_msgs::PoseWithCovarianceStamped vision;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_dt(ros::Time last);                                                        //获取时间间隔
void printf_info();     
void pose_pub_timer_cb(const ros::TimerEvent& TE);                        //ros::Timer的回调函数                                                                  //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void laser_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    //确定是cartographer发出来的/tf信息
    //有的时候/tf这个消息的发布者不止一个
    if (msg->transforms.size()>1 && msg->transforms[1].header.frame_id == "carto_odom")
    {
        laser = msg->transforms[1];
        //laser = msg->transforms[0];

        float dt_laser;

        dt_laser = (laser.header.stamp.sec - laser_last.header.stamp.sec) + (laser.header.stamp.nsec - laser_last.header.stamp.nsec)/10e9;

        //这里需要做这个判断是因为cartographer发布位置时有一个小bug，ENU到NED不展开讲。
        if (dt_laser != 0)
        {
            //位置 xy  [将解算的位置从laser坐标系转换至ENU坐标系]???
            pos_drone_laser[0]  = laser.transform.translation.x;
            pos_drone_laser[1]  = laser.transform.translation.y;

            // Read the Quaternion from the Carto Package [Frame: Laser[ENU]]
            Eigen::Quaterniond q_laser_enu(laser.transform.rotation.w, laser.transform.rotation.x, laser.transform.rotation.y, laser.transform.rotation.z);

            q_laser = q_laser_enu;

            // Transform the Quaternion to Euler Angles
            Euler_laser = quaternion_to_euler(q_laser);
        }
        laser_last = laser;
    }
}

void t265_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.frame_id == "camera_odom_frame")
    {
        pos_drone_t265[0] = msg->pose.pose.position.x;
        pos_drone_t265[1] = msg->pose.pose.position.y;
        //pos_drone_t265[2] = msg->pose.pose.position.z;
        pos_drone_t265[2] = pos_drone_fcu[2];
        Eigen::Quaterniond q_t265_enu(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        q_t265 = q_t265_enu;
        Euler_t265 = quaternion_to_euler(q_t265);
    }
}

void uwb_tagCallback(const nlink_parser::LinktrackNodeframe2 &msg)
{
    //ROS_INFO("msg LinktrackTagframe0 received,systemTime: %d", msg.systemTime);
    pos_drone_uwb[0] = msg.pos[0];
    pos_drone_uwb[1] = msg.pos[1];
    Eigen::Quaterniond q_uwb_enu(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
    q_uwb = q_uwb_enu;
    Euler_uwb = quaternion_to_euler(q_uwb);
}

void tfmini_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    tfmini_raw = msg->range;
}

//当无人机倾斜时,计算映射出的垂直高度
double vrt_h_map(const double& tfmini_raw,const double& roll,const double& pitch)
{
    //已知斜边,横滚角,俯仰角计算 映射出的垂直高度
    double vrt_h = tfmini_raw*cos(atan(sqrt( tan(roll)*tan(roll) + tan(pitch)*tan(pitch) )));
    //精度只取小数点后两位
    vrt_h = double(int(vrt_h*100)/100.0);
    return vrt_h;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    q_fcu = q_fcu_enu;

    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
   // Transform the Quaternion from ENU to NED
   // Eigen::Quaterniond q_ned = transform_orientation_enu_to_ned( transform_orientation_baselink_to_aircraft(q_fcu_enu) );
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");
    nh.param<int>("pose_source", pose_source, 0); //0---cartographer 1---t265 2---uwb
    // 【订阅】cartographer估计位置
    ros::Subscriber laser_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1000, laser_cb);
    // 【订阅】t265估计位置
    ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1000, t265_cb);
    // 【订阅】uwb估计位置
    ros::Subscriber sub = nh.subscribe("/nlink_linktrack_nodeframe2", 1000, uwb_tagCallback);

    // 【订阅】tf mini的数据
    ros::Subscriber tfmini_sub = nh.subscribe<sensor_msgs::Range>("/TFmini/TFmini", 100, tfmini_cb);

    // 【订阅】无人机当前位置 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系 这里订阅仅作比较用
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#??), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose", 100);
    ros::Timer pose_pub_timer = nh.createTimer(ros::Duration(1.0/20.0),pose_pub_timer_cb);
    // 频率
    ros::Rate rate(20.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        if(ready_for_pub)
        {
          vision_pub.publish(vision);
          ready_for_pub = false;
          printf_info();
        }
        rate.sleep();
    }

    return 0;

}

void pose_pub_timer_cb(const ros::TimerEvent& TE)
{
    //source -from cartographer
    if (pose_source == 0)   
    {
        pos_drone_laser[2] = pos_drone_fcu[2];
        vision.pose.position.x = pos_drone_laser[0];
        vision.pose.position.y = pos_drone_laser[1];
        vision.pose.position.z = pos_drone_laser[2];

        vision.pose.orientation.x = q_laser.x();
        vision.pose.orientation.y = q_laser.y();
        vision.pose.orientation.z = q_laser.z();
        vision.pose.orientation.w = q_laser.w();
        vision.header.stamp = TE.current_real;
    }
    //source -from t265
    if (pose_source == 1)
    {
        //pos_drone_t265[2] = pos_drone_fcu[2];
        vision.pose.position.x = pos_drone_t265[0];
        vision.pose.position.y = pos_drone_t265[1];
        vision.pose.position.z = pos_drone_t265[2];

        vision.pose.orientation.x = q_t265.x();
        vision.pose.orientation.y = q_t265.y();
        vision.pose.orientation.z = q_t265.z();
        vision.pose.orientation.w = q_t265.w();
        vision.header.stamp = TE.current_real;
    }
    //source -from uwb
    if (pose_source == 2)
    {
        pos_drone_uwb[2] = pos_drone_fcu[2];
        vision.pose.position.x = pos_drone_uwb[0];
        vision.pose.position.y = pos_drone_uwb[1];
        vision.pose.position.z = pos_drone_uwb[2];

        vision.pose.orientation.x = q_uwb.x();
        vision.pose.orientation.y = q_uwb.y();
        vision.pose.orientation.z = q_uwb.z();
        vision.pose.orientation.w = q_uwb.w();
        vision.header.stamp = TE.current_real;
    }
    ready_for_pub = true;
}


//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    if (pose_source == 0)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>Laser Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_laser [X Y Z] : " << pos_drone_laser[0] << " [ m ] "<< pos_drone_laser[1] <<" [ m ] "<< pos_drone_laser[2] <<" [ m ] "<<endl;
        cout << "Euler_vlaser[Yaw] : " << Euler_laser[2] * 180/M_PI<<" [deg]  "<<endl;
    }
    if (pose_source == 1)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>T265 Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_t265 [X Y Z] : " << pos_drone_t265[0] << " [ m ] "<< pos_drone_t265[1] <<" [ m ] "<< pos_drone_t265[2] <<" [ m ] "<<endl;
        cout << "Euler_t265[Yaw] : " << Euler_t265[2] * 180/M_PI<<" [deg]  "<<endl;       
    }
    if (pose_source == 2)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>Uwb Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_uwb [X Y Z] : " << pos_drone_uwb[0] << " [ m ] "<< pos_drone_uwb[1] <<" [ m ] "<< pos_drone_uwb[2] <<" [ m ] "<<endl;
        cout << "Euler_uwb[Yaw] : " << Euler_uwb[2] * 180/M_PI<<" [deg]  "<<endl;       
    }

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>FCU Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1] <<" [ m ] "<< pos_drone_fcu[2] <<" [ m ] "<<endl;
    cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1] <<" [m/s] "<< vel_drone_fcu[2] <<" [m/s] "<<endl;
    cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;
}
