#include <ros/ros.h>

#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>


using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}
float cal_dis(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}
px4_command::command Command_now;
//---------------------------------------正方形参数---------------------------------------------
float size_square; //正方形边长
float height_pantagon;                //飞行高度
float sleep_time;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "newstar");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    nh.param<float>("size_square", size_square, 1.0);
    nh.param<float>("height_square", height_pantagon, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    int check_flag;
    cout << "size_square: " << size_square << "[m]" << endl;
    cout << "height_square: " << height_pantagon << "[m]" << endl;
    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;
    if (check_flag != 1) {
        return -1;
    }

    int Arm_flag;
    cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
    cin >> Arm_flag;
    if (Arm_flag == 1) {
        Command_now.command = Arm;
        move_pub.publish(Command_now);
    } else {
        return -1;
    }

    int takeoff_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit " << endl;
    cin >> takeoff_flag;
    if (takeoff_flag != 1) {
        return -1;
    }

    int i = 0;
    int comid = 0;
    float abs_distance = 1e5;

    while (i < sleep_time*20) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 0----->takeoff" << endl;
        i++;
    }
    //top 1
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square / 2.0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 1----->move to top" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    //right-bottom 2
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square / 2.0;
        Command_now.pos_sp[1] = -size_square / 2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 2----->right-bottom"<<endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    //left-top 3

    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square / 10.0;
        Command_now.pos_sp[1] = size_square / 2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 3----->move to (left-top)" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    
    //right-up 4
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square / 10.0;
        Command_now.pos_sp[1] = -size_square / 2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 4----->move to (right-up)" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
        
    //left-bottom 5
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square / 2.0;
        Command_now.pos_sp[1] = size_square / 2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 5----->move to (left-bottom)" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    //up 6
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square / 2.0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 6----->move to (up)" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    
    //return 7
    abs_distance = 1e5;
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 7----->move to return" << endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }


    Command_now.command = Land;
    while (ros::ok()) {
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Land" << endl;
    }
    rate.sleep();
    cout << "Mission complete, exiting...." << endl;
    return 0;
}
