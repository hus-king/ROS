#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>
#include <algorithm>
#include <my_opencv_pkg/square_center.h>
#include <my_opencv_pkg/detector_bool.h>

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
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
my_opencv_pkg::square_center square_center;
my_opencv_pkg::detector_bool detect_msg;

float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float fly_height;
//--------------------------------------------算法相关--------------------------------------------------
float R_inside;                                                 //安全半径 [避障算法相关参数]
float distance_c,angle_c;                                       //最近障碍物距离 角度
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float target_angle;                                             //目标角度
float colision_tangent_angle;                                   //避障圆与目标点连线的切线角度
float colision_angle[2];                                       //两个切线方向
float vel_sp_body[2];                                           //总速度
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_ENU_all = 0.2;
float sleep_time;
float vel_sp_max;                                               //总速度限幅
px4_command::command Command_now;                               //发送给position_control.cpp的命令
float fx=554.3827;                                               //相机内参（待测）
float fy=554.3827;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
void printf();                                                                       //打印函数
void printf_param();
float cal_dis(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}                                                                 //打印各项参数以供检查
void cone_avoidance(float target_x,float target_y);
void v_control(float v, float newv[2], float target_angle);
void normalize_angle(float *angle);
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;
    int count;    //count = 359
    count = Laser.ranges.size();
    for(int i = 0; i < count; i++)
    {
           if(i+180>359) Laser.ranges[i]=Laser_tmp.ranges[i-180];
           else Laser.ranges[i]=Laser_tmp.ranges[i+180];
    }
    cal_min_distance();
}
//回调函数，当接收到 geometry_msgs::PoseStamped 类型的消息时被调用。
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
    //将接收到的消息(Mavros Package [Frame: ENU])存储在全局变量 pos_drone 中。  
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    //从消息中提取四元数，表示无人机在ENU坐标系下的姿态。
    q_fcu = q_fcu_enu;
    //将提取的四元数存储在全局变量 q_fcu 中。
    Euler_fcu = quaternion_to_euler(q_fcu);
    //将四元数转换为欧拉角，并存储在全局变量 Euler_fcu 中。
}

void square_cb(const my_opencv_pkg::square_center::ConstPtr& msg) {
    square_center = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);
    //【订阅】Lidar数据
    //ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);
    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // 【订阅】无人机的降落检测结果
    ros::Subscriber squre_sub = nh.subscribe<my_opencv_pkg::square_center>("/my_opencv_pkg/square_center", 10, square_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    // ros::Publisher flag_pub = nh.advertise<std_msgs::Bool>("/px4/flag", 10);
    ros::Publisher detect_pub = nh.advertise<my_opencv_pkg::detector_bool>("/my_opencv_pkg/detector_bool", 1);
    detect_msg.detect = false;
    detect_pub.publish(detect_msg);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0); //dyx
    nh.param<float>("target_y", target_y, 0.0); //dyx
    nh.param<float>("R_inside", R_inside, 0.6);
    nh.param<int>("range_min", range_min, 0);
    nh.param<int>("range_max", range_max, 0);
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);
    //打印现实检查参数
    printf_param();

    //check paramater
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;
    if(check_flag != 1) return -1;

    //check arm
    int Arm_flag;
    cout<<"Whether choose to Arm? 1 for Arm, 0 for quit"<<endl;
    cin >> Arm_flag;
    if(Arm_flag == 1)
    {
        Command_now.command = Arm;
        command_pub.publish(Command_now);
    }
    else return -1;

    //check takeoff
    int Take_off_flag;
    cout<<"Whether choose to Takeoff? 1 for Takeoff, 0 for quit"<<endl;
    cin >> Take_off_flag;
    int comid = 0;
    sleep_time = sleep_time * 20;
    float abs_distance = 1e5;
    
    while (pos_drone.pose.position.z < fly_height - 0.05)  //飞到指定高度   
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 1 -----> takeoff"<<endl;
        cout << "z = "<<pos_drone.pose.position.z<<endl;
        cout << "target = "<<fly_height<<endl;
    }

    //初值
    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        cone_avoidance(target_x,target_y);

        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0;

        float abs_distance;
        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
        if(abs_distance < 0.3){
            break;
        }
        command_pub.publish(Command_now);
        
        //打印
        printf();
        rate.sleep();
    }
    // 发布检测消息:
    
    detect_msg.detect = true; // 设置检测标志为true
    detect_pub.publish(detect_msg);
    // 发布消息
    ros::Duration(3.0).sleep(); // 等待一段时间以确保消息被发送

    // 获取square_center消息:
    ros::spinOnce();
    cout << "square_center.x: " << square_center.x << endl;
    cout << "square_center.y: " << square_center.y << endl;
    //square_center.x与square_center.y为现在飞机中心的像素坐标
    //目标位置 center = (320,283)
    int dx = square_center.x - 320;
    int dy = square_center.y - 283;
    //存疑，需要实操验证
    int adjust_flag = 0;
    if (abs(dx) > 10 || abs(dy) > 10) adjust_flag = 1; //如果偏差大于10，进行调整
    if (abs(dx) > 30 || abs(dy) > 30) adjust_flag = 0; //如果偏差大于30，不调整
    cout << "adjust_flag = "<<adjust_flag<<endl;

    //dx,dy为像素差值
    //根据像素差值计算ENU坐标系下的差值
    float adjust_x = (dx * fly_height) / fx; // East方向偏移
    float adjust_y = (dy * fly_height) / fy; // North方向偏移
    
    int i = 0;
    //while (ros::ok()) //测试用
    while (i < sleep_time / 4 && adjust_flag == 1) //2.5s
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = target_x + adjust_x; // 目标位置加上调整值
        Command_now.pos_sp[1] = target_y + adjust_y; // 目标位置加上调整值
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 3 -----> adjust"<<endl;
        cout << "x = "<<pos_drone.pose.position.y<<endl;
        cout << "y = "<<pos_drone.pose.position.x<<endl;
        cout << "target_x = "<<target_x<<endl;
        cout << "target_y = "<<target_y<<endl;
        cout << "adjust_x = "<<adjust_x<<endl;
        cout << "adjust_y = "<<adjust_y<<endl;
        i++;
    }
    Command_now.command = Land;
    while (ros::ok()) {
        move_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce();
        cout << "Land" << endl;
    }
    rate.sleep();
    cout << "Mission complete, exiting...." << endl;
    return 0;
}

//计算前后左右四向最小距离
void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    
    for (int i = range_min; i <= range_max; i++)
    {
        if(Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }
    angle_c = angle_c + Euler_fcu[2] * 180.0/M_PI;
    normalize_angle(&angle_c);
}


void cone_avoidance(float target_x,float target_y){
    //2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_inside ) flag_collision_avoidance.data = false;
    else{
        flag_collision_avoidance.data = true;
    }
    target_angle = atan2(target_y - pos_drone.pose.position.y, target_x - pos_drone.pose.position.x);
    target_angle = target_angle * 180.0 / M_PI; // 将弧度转换为度数

    colision_angle[0] = angle_c - 90;
    colision_angle[1] = angle_c + 90;
    colision_angle[0] = abs(target_angle - colision_angle[0]);
    colision_angle[1] = abs(target_angle - colision_angle[1]);
    if(colision_angle[0] > 180) colision_angle[0] = 360 - colision_angle[0];
    if(colision_angle[1] > 180) colision_angle[1] = 360 - colision_angle[1];
    if(colision_angle[0] > colision_angle[1]) colision_tangent_angle = angle_c + 90;
    else colision_tangent_angle = angle_c - 90;
    normalize_angle(&colision_tangent_angle);
    //通过与目标角度比较来选取合适切线角度

    //3. 计算速度
    if(flag_collision_avoidance.data == true){
        v_control(vel_sp_ENU_all, vel_sp_ENU, colision_tangent_angle);
    }
    else{
        v_control(vel_sp_ENU_all, vel_sp_ENU, target_angle);
    }
}
void printf()
{
    cout << "Point 2 -----> moving"<<endl;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Minimun_distance : "<<endl;
    cout << "Distance : " << distance_c << " [m] "<<endl;
    cout << "angle_c :    " << angle_c    << " [du] "<<endl;
    if(flag_collision_avoidance.data == true )
    {
        cout << "Cone avoidance Enabled "<<endl;
    }
    else
    {
        cout << "Collision avoidance Disabled "<<endl;
    }
    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] "<<endl;
    cout << "angle_c : " << angle_c << " [du] "<<endl;
    cout << "target_angle : " << target_angle<< " [du] "<<endl;
    cout << "colision_tangent_angle : " << colision_tangent_angle << " [du] "<<endl;
    cout << "pos_drone : " << pos_drone.pose.position.x << " [m] "<< pos_drone.pose.position.y << " [m] "<< pos_drone.pose.position.z << " [m] "<<endl;
    cout << "target_x : " << target_x << " [m] "<< "target_y : " << target_y << " [m] "<<endl;
    cout << "Euler_fcu : " << Euler_fcu[2] * 180.0/M_PI << " [du] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;
    cout << "R_inside : "<< R_inside << endl;
    cout << "vel_sp_ENU_all : "<< vel_sp_ENU_all << endl;
    cout << "range_min : "<< range_min << endl;
    cout << "range_max : "<< range_max << endl;
    cout << "fly_height: "<<fly_height<<endl;
    cout << "sleep_time "<<sleep_time<<endl;
}
void v_control(float v, float newv[2], float target_angle) {
    // 将角度从度转换为弧度
    float angle = target_angle * M_PI / 180.0;
    // 计算新的速度分量
    newv[0] = v * cos(angle);
    newv[1] = v * sin(angle);
}
void normalize_angle(float *angle) {
    // 将角度调整到 -180 到 180 度范围内
    while (*angle > 180.0) {
        *angle -= 360.0;
    }
    while (*angle < -180.0) {
        *angle += 360.0;
    }
}