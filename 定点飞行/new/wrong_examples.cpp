#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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
    ros::init(argc, argv, "square");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    nh.param<float>("size_square", size_square, 1.0);
    nh.param<float>("height_pantegon", height_pantagon, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    int check_flag;
    cout << "size_square: "<<size_square<<"[m]"<<endl;
    cout << "height_square: "<<height_pantagon<<"[m]"<<endl;
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1)
    {
        return -1;
    }

    //check arm
    int Arm_flag;
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag == 1)
    {
        Command_now.command = Arm;
        move_pub.publish(Command_now);
    }
    else return -1;

    int takeoff_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit "<<endl;
    cin >> takeoff_flag;
    if(takeoff_flag != 1)
    {
        return -1;
    }
    int i=0;
    int comid = 0;
    i = 0;
    int abs_distance;
    
    //takeoff
    while (i < sleep_time)
    {
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
        cout << "Point 0----->takeoff"<<endl;
        i++;
    }

    //point left bottom
    while(abs_distance > 0.3)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = -size_square/2.0;
        Command_now.pos_sp[1] = -size_square/2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 0----->takeoff"<<endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }

    //point left top
    while(abs_distance > 0.3)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square/2.0;
        Command_now.pos_sp[1] = -size_square/2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 0----->takeoff"<<endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }

    //point right top
    while(abs_distance > 0.3)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = size_square/2.0;
        Command_now.pos_sp[1] = size_square/2.0;
        Command_now.pos_sp[2] = height_pantagon;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 0----->takeoff"<<endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }

    //降落
    Command_now.command = Land;
    while (ros::ok())
    {
      move_pub.publish(Command_now);
      rate.sleep();
      cout << "Land"<<endl;
    }
    rate.sleep();
    cout << "Mission complete, exiting...."<<endl;
    return 0;
}