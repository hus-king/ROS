#include <ros/ros.h>
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <yolov5_ros_msgs/BoundingBoxes.h> //目标检测
#include <math_utils.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <std_msgs/Int8.h>
#include <string>
#include <map>

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

enum DetectMode {
    Off,
    Door,
    Qr_code,
};
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float point1_x;
float point1_y;
float point2_x;
float point2_y;
float point3_x;
float point3_y;
float point4_x;
float point4_y;
float code1_x;
float code1_y;
float code2_x;
float code2_y;
float code3_x;
float code3_y;
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float last_time = 0;
float fly_height;
//--------------------------------------------运动算法相关--------------------------------------------------
float R_outside, R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c, angle_c;                                       //最近障碍物距离 角度
float distance_cx, distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_ENU[2];
float vel_collision_max;                                        //躲避障碍部分速度限幅
float vel_collision_total;
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_ENU[2];
float vel_track_total;                                          //追踪总速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
float angle_target_ENU;                                             //目标角
float angle_target_ENU_origin;                                      //起始目标角
int angle_target;                                                 //机体坐标系下目标角
int angle_diff;
float angle_judge;                                                  //判断角
float turn_angle = 0;                                               //转向角
float turn_angle1;
float turn_angle2;
float turn_angle3;
float turn_angle4;
float yaw_tolerance;                                            //误差容忍
float turn_angle_radians = 0;                                   //弧度制转向角
float yaw_tolerance_radians = 0;                                //误差弧度制角
float turn_sp;                                                  //转向速度
float takeoff_time;
float hover_time;
float detect_time;                                              //检测二维码时长    
float nearly_stop;
int diff_last = 0;
int diff_now = 0;
//--------------------------------------------运动输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_total;                                             //当前总速度
float vel_sp_max;                                               //总速度限幅
int flag_direction;                                              //避障的方向判断
bool flag_inline;
float deviation_angle;
//--------------------------------------------图像识别--------------------------------------------------
int detect_num;                                             //yolov5发布的检测到的物体数目
map<string, int> qr_map;
yolov5_ros_msgs::BoundingBox yolov5_box;                      //用于模式4只用识别一张图的情况
yolov5_ros_msgs::BoundingBoxes yolov5_boxes;                  //用于模式5需要识别三张图的情况
float fx = 554.3827;                                               //相机内参
float fy = 554.3827;
float cx = 320;
float cy = 240;
float pic_target[2];                                            //模式4的图像中心ENU坐标
DetectMode detect_mode = Off;
string qr_class;

px4_command::command Command_now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data[2], float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
void collision_avoidance(float target_x, float target_y);
int calculate_angle_target(float target_x, float target_y);
int fly_direction();
bool judge_inline();
void detect_nav();
void land_center(float& target_x, float& target_y);
string detect_result();



// 【坐标系旋转函数】- 机体系到ENU系
// input是机体系,output是ENU系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;
    int count;    //count = 359
    count = Laser.ranges.size();

    //剔除inf的情况
    for (int i = 0; i < count; i++)
    {
        //判断是否为inf
        int a = isinf(Laser_tmp.ranges[i]);
        //如果为inf，则赋值上一角度的值
        if (a == 1)
        {
            if (i == 0)
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[count - 1];
            }
            else
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[i - 1];
            }
        }

    }
    for (int i = 0; i < count; i++)
    {
        if (i + 180 > 359) Laser.ranges[i] = Laser_tmp.ranges[i - 180];
        else Laser.ranges[i] = Laser_tmp.ranges[i + 180];
        //cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
    }
    //cout<<"//////////////"<<endl;
    //计算前后左右四向最小距离
    cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
    // 按ZYX顺序获取欧拉角，可按需调整顺序
}

//yolov5发布的检测到的图片数目
void yolov5_found_cb(const std_msgs::Int8::ConstPtr& msg)
{
    detect_num = msg->data;
}

//yolov5发布的检测到的所有图片的坐标
void yolov5_box_cb(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& msg)   //数组中的元素不会保留上一次(帧)的检测结果
{
    yolov5_boxes = *msg; // 存储所有边界框
    

    // 处理门的检测
    switch (detect_mode)
    {
    case Door:
        if (msg->bounding_boxes.size() > 0 && msg->bounding_boxes[0].Class == "door")
        {
            cout << "Found door!" << endl;
            yolov5_box = msg->bounding_boxes[0];                       // 存储第一个边界框
            detect_mode = Off;
        }
        break;
    case Qr_code:
        // 处理二维码的检测（用于 point3）
        if (msg->bounding_boxes.size() > 0)
        {
            qr_map[msg->bounding_boxes[0].Class]++;
            cout<<"Detected :"<<msg->bounding_boxes[0].Class<<endl;
        }
        break;
    default:
        break;
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser", 1000, lidar_cb);
    //【订阅】yolov5数据
    ros::Subscriber yolov5_box_sub = nh.subscribe<yolov5_ros_msgs::BoundingBoxes>("/yolov5_ros/bounding_boxes", 1, yolov5_box_cb);//dyx
    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //读取参数表中的参数
    nh.param<float>("point1_x", point1_x, 0.0);
    nh.param<float>("point1_y", point1_y, 0.0);
    nh.param<float>("point2_x", point2_x, 0.0);
    nh.param<float>("point2_y", point2_y, 0.0);
    nh.param<float>("point3_x", point3_x, 0.0);
    nh.param<float>("point3_y", point3_y, 0.0);
    nh.param<float>("point4_x", point4_x, 0.0);
    nh.param<float>("point4_y", point4_y, 0.0);

    nh.param<float>("code1_x", code1_x, 0.0);
    nh.param<float>("code1_y", code1_y, 0.0);
    nh.param<float>("code2_x", code2_x, 0.0);
    nh.param<float>("code2_y", code2_y, 0.0);
    nh.param<float>("code3_x", code3_x, 0.0);
    nh.param<float>("code3_y", code3_y, 0.0);

    nh.param<float>("R_outside", R_outside, 2);
    nh.param<float>("R_inside", R_inside, 1);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_track_max", vel_track_max, 0.5);

    nh.param<float>("p_R", p_R, 0.0);
    nh.param<float>("p_r", p_r, 0.0);

    nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("takeoff_time", takeoff_time, 15.0);
    nh.param<float>("detect_time", detect_time, 5.0);
    nh.param<float>("hover_time", hover_time, 3.0);


    nh.param<float>("turn_angle1", turn_angle1, 0);
    nh.param<float>("turn_angle2", turn_angle2, 0);
    nh.param<float>("turn_angle3", turn_angle3, 0);
    nh.param<float>("turn_angle4", turn_angle4, 0);
    nh.param<float>("yaw_tolerance", yaw_tolerance, 5);
    nh.param<float>("turn_sp", turn_sp, 1);

    nh.param<float>("nearly_stop", nearly_stop, 0.01);
    nh.param<float>("angle_judge", angle_judge, 1);

    // 相机内参
    nh.param<float>("fx", fx, 554.3827);
    nh.param<float>("fy", fy, 554.3827);
    nh.param<float>("cx", cx, 320.0);
    nh.param<float>("cy", cy, 240.0);

    turn_angle_radians = (turn_angle / 180) * 3.1415926;
    yaw_tolerance_radians = (yaw_tolerance / 180) * 3.1415926;              //转换成弧度制的角
    //打印现实检查参数
    printf_param();

    //check paramater
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;
    if (check_flag != 1) return -1;

    //check arm
    int Arm_flag;
    cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
    cin >> Arm_flag;
    if (Arm_flag == 1)
    {
        Command_now.command = Arm;
        command_pub.publish(Command_now);
    }
    else return -1;

    //check takeoff
    int i = 0;
    int comid = 0;
    int Take_off_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit" << endl;
    cin >> Take_off_flag;
    if (Take_off_flag == 1)
    {
        i = 0;
        while (i < takeoff_time * 20)
        {
            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = fly_height;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;
            move_pub.publish(Command_now);
            rate.sleep();
            cout << "Point 0----->takeoff" << endl;
            i++;
        }

    }
    else return -1;

    while (ros::ok())                                           //飞往point1
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point1_x;
        Command_now.pos_sp[1] = point1_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point1_x : " << point1_x << endl;
        cout << "point1_y : " << point1_y << endl;

        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        if (abs_distance < 0.1)
        {
            cout << "Having reached the point1" << endl;
            break;
        }
    }

    i = 0;
    while (i < hover_time * 20)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point1_x;
        Command_now.pos_sp[1] = point1_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Hovering" << endl;
        i++;
    }

    //右转90度
    while (ros::ok()) {
        ros::spinOnce();
        turn_angle = turn_angle1;
        turn_angle_radians = (turn_angle / 180) * 3.1415926;
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point1_x;                                          //保持位置不变
        Command_now.pos_sp[1] = point1_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = std::max(Command_now.yaw_sp - turn_sp, turn_angle);    //转向速度控制
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        double current_yaw = Euler_fcu(2);
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        cout << "------------->>Turning<<-----------" << endl;
        cout << "current_yaw : " << current_yaw / 3.1415926 * 180 << endl;
        cout << "target_angle1 : " << turn_angle << endl;
        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point1_x : " << point1_x << endl;
        cout << "point1_y : " << point1_y << endl;
        if (abs(current_yaw - turn_angle_radians) < yaw_tolerance_radians && abs_distance < 0.1)                 // 转向完成的判断
        {
            cout << "----------->>Finished<<------------" << endl;
            break;
        }

    }
     
    ////识别门
    //detect_mode = Door;
    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //    detect_nav();
    //    Command_now.command = Move_ENU;
    //    Command_now.sub_mode = 0;
    //    Command_now.pos_sp[0] = point1_x;
    //    Command_now.pos_sp[1] = point1_y;
    //    Command_now.pos_sp[2] = fly_height;
    //    Command_now.yaw_sp = turn_angle;
    //    Command_now.comid = comid;
    //    comid++;
    //    move_pub.publish(Command_now);
    //    rate.sleep();
    //    cout << "Detecting Door" << endl;
    //    if (detect_mode == Off)break;
    //}
    //
    //point2_x = pic_target[0];
    //point2_y = pic_target[1] - 1;

    while (ros::ok())                                           //飞往point2，识别门并穿过
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point2_x;
        Command_now.pos_sp[1] = point2_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance;

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point2_x : " << point2_x << endl;
        cout << "point2_y : " << point2_y << endl;


        abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        if (abs_distance < 0.1)
        {
            cout << "Having reached the point2" << endl;
            break;
        }
    } 
 
    i = 0;
    while (i < hover_time * 20)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point2_x;
        Command_now.pos_sp[1] = point2_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Hovering" << endl;
        i++;
    }

    while (ros::ok()) {         //再右转90度
        ros::spinOnce();
        turn_angle = turn_angle2;
        turn_angle_radians = (turn_angle / 180) * 3.1415926;
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point2_x;                                          //保持位置不变
        Command_now.pos_sp[1] = point2_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = std::max(Command_now.yaw_sp - turn_sp, turn_angle);    //转向速度控制
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        double current_yaw = Euler_fcu(2);
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        cout << "------------->>Turning<<-----------" << endl;
        cout << "current_yaw : " << current_yaw / 3.1415926 * 180 << endl;
        cout << "target_angle2 : " << turn_angle << endl;
        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point2_x : " << point2_x << endl;
        cout << "point2_y : " << point2_y << endl;
        if (abs(current_yaw - turn_angle_radians) < yaw_tolerance_radians && abs_distance < 0.1)                 // 转向完成的判断
        {
            cout << "----------->>Finished<<------------" << endl;
            break;
        }

    }

    //各项速度初值
    vel_track[0] = 0;
    vel_track[1] = 0;

    vel_collision[0] = 0;
    vel_collision[1] = 0;

    vel_sp_body[0] = 0;
    vel_sp_body[1] = 0;

    vel_sp_ENU[0] = 0;
    vel_sp_ENU[1] = 0;

    flag_land = 0;


    while (ros::ok())               //飞向point3
    {
        ros::spinOnce();
        collision_avoidance(point3_x, point3_y);
        Command_now.command = Move_ENU;             //机体系下移动，机头为x方向
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2;                   // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] = vel_sp_ENU[0];
        Command_now.vel_sp[1] = vel_sp_ENU[1];
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - point3_x) * (pos_drone.pose.position.x - point3_x) + (pos_drone.pose.position.y - point3_y) * (pos_drone.pose.position.y - point3_y));
        if (abs_distance < 0.1)
        {
            cout << "Having reached the point3" << endl;
            break;
        }
        //打印
        printf();
    }

    
    detect_mode = Qr_code;
    while (ros::ok())          //左转90度
    {
        ros::spinOnce();
        turn_angle = turn_angle3;
        turn_angle_radians = (turn_angle / 180) * 3.1415926;
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point3_x;                                          //保持位置不变
        Command_now.pos_sp[1] = point3_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = std::min(Command_now.yaw_sp + turn_sp, turn_angle);    //转向速度控制
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        double current_yaw = Euler_fcu(2);
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        cout << "------------->>Turning<<-----------" << endl;
        cout << "current_yaw : " << current_yaw / 3.1415926 * 180 << endl;
        cout << "target_angle3 : " << turn_angle << endl;
        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point3_x : " << point3_x << endl;
        cout << "point3_y : " << point3_y << endl;
        if (abs(current_yaw - turn_angle_radians) < yaw_tolerance_radians && abs_distance < 0.1)                 // 转向完成的判断
        {
            cout << "----------->>Finished<<------------" << endl;
            break;
        }

    }

    //悬停并识别二维码
    i = 0;
    while (i < detect_time * 20)
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point3_x;
        Command_now.pos_sp[1] = point3_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        i++;
        cout << "Detecting time :" << i/20.0<<"(s)"<<endl;
    }

    qr_class = detect_result();
    qr_map.clear();
    detect_mode = Off;

    
    //各项速度初值
    vel_track[0] = 0;
    vel_track[1] = 0;

    vel_collision[0] = 0;
    vel_collision[1] = 0;

    vel_sp_body[0] = 0;
    vel_sp_body[1] = 0;

    vel_sp_ENU[0] = 0;
    vel_sp_ENU[1] = 0;

    flag_land = 0;

    while (ros::ok())               //飞向point4
    {
        ros::spinOnce();
        collision_avoidance(point4_x, point4_y);
        Command_now.command = Move_ENU;
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2;
        Command_now.vel_sp[0] = vel_sp_ENU[0];
        Command_now.vel_sp[1] = vel_sp_ENU[1];
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - point4_x) * (pos_drone.pose.position.x - point4_x) + (pos_drone.pose.position.y - point4_y) * (pos_drone.pose.position.y - point4_y));
        if (abs_distance < 0.1)
        {
            cout << "Having reached the point4" << endl;
            break;
        }
        //打印
        printf();
    }

    i = 0;
    while (i < hover_time * 20)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point4_x;
        Command_now.pos_sp[1] = point4_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Hovering" << endl;
        i++;
    }

    while (ros::ok()) {         //再左转90度
        ros::spinOnce();
        turn_angle = turn_angle4;
        turn_angle_radians = (turn_angle / 180) * 3.1415926;
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = point4_x;                                          //保持位置不变
        Command_now.pos_sp[1] = point4_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = std::min(Command_now.yaw_sp + turn_sp, turn_angle);    //转向速度控制
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        double current_yaw = Euler_fcu(2);
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));
        cout << "------------->>Turning<<-----------" << endl;
        cout << "current_yaw : " << current_yaw / 3.1415926 * 180 << endl;
        cout << "target_angle4 : " << turn_angle << endl;
        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "point4_x : " << point4_x << endl;
        cout << "point4_y : " << point4_y << endl;
        if (abs(current_yaw - turn_angle_radians) < yaw_tolerance_radians && abs_distance < 0.1)                 // 转向完成的判断
        {
            cout << "----------->>Finished<<------------" << endl;
            break;
        }
    }
     
    detect_mode = Qr_code;
    while (ros::ok())                                           //飞往code1,开始识别第一个二维码
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code1_x;
        Command_now.pos_sp[1] = code1_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "code1_x : " << code1_x << endl;
        cout << "code1_y : " << code1_y << endl;

        if (abs_distance < 0.1)break;
    }
    
    i = 0;
    while (i < detect_time * 20)
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code1_x;
        Command_now.pos_sp[1] = code1_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        i++;
        cout << "target_class: " << qr_class << endl;
        cout << "Detecting time :" << i/20.0<<"(s)"<<endl;
    }
    if (!qr_map.empty()) {
        string now_class = detect_result();
        if (now_class == qr_class) {
            detect_mode = Off;
        }
    }
    qr_map.clear();

    if (detect_mode == Off)
    {
        /*float center_x, center_y;
        land_center(center_x, center_y);*/
        while (ros::ok())                                           //对准中心
        {
            ros::spinOnce();
            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = code1_x;
            Command_now.pos_sp[1] = code1_y;
            Command_now.pos_sp[2] = fly_height;
            Command_now.yaw_sp = turn_angle;
            Command_now.comid = comid;
            comid++;
            move_pub.publish(Command_now);
            rate.sleep();
            float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

            cout << "pos_x: " << pos_drone.pose.position.x << endl;
            cout << "pos_y: " << pos_drone.pose.position.y << endl;
            cout << "code1_x : " << code1_x << endl;
            cout << "code1_y : " << code1_y << endl;

            if (abs_distance < 0.1 || flag_land == 1) {
                Command_now.command = 3;     //Land
                flag_land = 1;
            }
            if (flag_land == 1) Command_now.command = Land;
            command_pub.publish(Command_now);
            rate.sleep();
        }

        return 0;
    }

    while (ros::ok())                                           //飞往第二个二维码
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code2_x;
        Command_now.pos_sp[1] = code2_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "code2_x : " << code2_x << endl;
        cout << "code2_y : " << code2_y << endl;

        if (abs_distance < 0.1)break;
    }
    i = 0;
    while (i < detect_time * 20)
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code2_x;
        Command_now.pos_sp[1] = code2_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        i++;
        cout << "target_class: " << qr_class << endl;
        cout << "Detecting time :" << i/20.0<<"(s)"<<endl;
    }
    if (!qr_map.empty()) 
    {
        string now_class = detect_result();
        if (now_class == qr_class) 
        {
            detect_mode = Off;
        }
    }
    qr_map.clear();

    if (detect_mode == Off)
    {
       /* float center_x, center_y;
        land_center(center_x, center_y);*/
        while (ros::ok())                                  
        {
            ros::spinOnce();
            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = code2_x;
            Command_now.pos_sp[1] = code2_y;
            Command_now.pos_sp[2] = fly_height;
            Command_now.yaw_sp = turn_angle;
            Command_now.comid = comid;
            comid++;
            move_pub.publish(Command_now);
            rate.sleep();
            float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

            cout << "pos_x: " << pos_drone.pose.position.x << endl;
            cout << "pos_y: " << pos_drone.pose.position.y << endl;
            cout << "code2_x : " << code2_x << endl;
            cout << "code2_y : " << code2_y << endl;

            if (abs_distance < 0.1 || flag_land == 1) {
                Command_now.command = 3;     //Land
                flag_land = 1;
            }
            if (flag_land == 1) Command_now.command = Land;
            command_pub.publish(Command_now);
            rate.sleep();
        }

        return 0;
    }

    while (ros::ok())                                           //飞往第三个二维码
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code3_x;
        Command_now.pos_sp[1] = code3_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "code3_x : " << code3_x << endl;
        cout << "code3_y : " << code3_y << endl;

        if (abs_distance < 0.1)break;
    }

    //float center_x, center_y;                              //最后一个直接找中心降落
    //land_center(center_x, center_y);
    while (ros::ok())
    {
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = code3_x;
        Command_now.pos_sp[1] = code3_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        Command_now.comid = comid;
        comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        float abs_distance = sqrt((pos_drone.pose.position.x - Command_now.pos_sp[0]) * (pos_drone.pose.position.x - Command_now.pos_sp[0]) + (pos_drone.pose.position.y - Command_now.pos_sp[1]) * (pos_drone.pose.position.y - Command_now.pos_sp[1]));

        cout << "pos_x: " << pos_drone.pose.position.x << endl;
        cout << "pos_y: " << pos_drone.pose.position.y << endl;
        cout << "code3_x : " << code3_x << endl;
        cout << "code3_y : " << code3_y << endl;

        if (abs_distance < 0.1 || flag_land == 1) {
            Command_now.command = 3;     //Land
            flag_land = 1;
        }
        if (flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        rate.sleep();
    }

    return 0;
}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//计算全方位距离障碍物最小距离
void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    for (int i = range_min; i <= range_max; i++)
    {
        if (Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }
}

//饱和函数，用于限速
float satfunc(float data[2], float Max)
{
    float total = sqrt(data[0] * data[0] + data[1] * data[1]);
    if (total > Max) {
        data[0] = Max * (data[0] / total);
        data[1] = Max * (data[1] / total);
        total = Max;
    }

    return total;
}

void collision_avoidance(float target_x, float target_y)
{


    //2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside)
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    //3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);       //ENU坐标系下的速度
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);


    //速度限幅
    vel_track_total = satfunc(vel_track, vel_track_max);

    vel_collision_ENU[0] = 0;
    vel_collision_ENU[1] = 0;

    //4. 避障策略
    if (flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(angle_c / 180 * 3.1415926);         //障碍物极坐标转化为机体坐标系下的x,y坐标
        distance_cy = distance_c * sin(angle_c / 180 * 3.1415926);

        float F_c = 0.0;

        vel_collision[0] = 0;
        vel_collision[1] = 0;

        //小幅度抑制移动速度
        if (distance_c > R_inside && distance_c <= R_outside)
        {
            F_c = p_R * (R_outside - distance_c);
        }

        //大幅度抑制移动速度
        if (distance_c <= R_inside)
        {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }

        //改变避障速度
        vel_collision[0] = -F_c * distance_cx / distance_c;   //x方向
        vel_collision[1] = -F_c * distance_cy / distance_c;   //y方向

        angle_target = calculate_angle_target(target_x, target_y);
        flag_direction = fly_direction();
        flag_inline = judge_inline();

        if (flag_inline == true) {

            deviation_angle = angle_c - 90 * flag_direction;

            // 调整速度分量，使无人机沿着垂直偏离方向移动
            vel_collision[0] = vel_collision[0] + cos(deviation_angle / 180 * 3.1415926) * vel_sp_max;
            vel_collision[1] = vel_collision[1] + sin(deviation_angle / 180 * 3.1415926) * vel_sp_max;
        }

        vel_collision_total = satfunc(vel_collision, vel_collision_max);

        rotation_yaw(Euler_fcu(2), vel_collision, vel_collision_ENU);                  //避障速度从机体坐标系转到ENU坐标系

    }

    vel_sp_ENU[0] = vel_track[0] + vel_collision_ENU[0];
    vel_sp_ENU[1] = vel_track[1] + vel_collision_ENU[1];

    vel_sp_total = satfunc(vel_sp_ENU, vel_sp_max);

}

void printf()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Minimun_distance : " << distance_c << " [m] " << endl;
    cout << "angle_c :    " << angle_c << " [du] " << endl;
    cout << "angle_target :    " << angle_target << " [du] " << endl;
    cout << "angle_diff :    " << angle_diff << " [du] " << endl;
    cout << "distance_c :    " << distance_c << " [m] " << endl;
    if (flag_collision_avoidance.data == true)
    {
        cout << "Collision avoidance Enabled " << endl;
    }
    else
    {
        cout << "Collision avoidance Disabled " << endl;
    }

    if (flag_inline)
    {
        cout << "Deviation Enabled " << endl;
        cout << "Deviation_angle:" << deviation_angle << " [du] " << endl;
    }
    else
    {
        cout << "Deviation Disabled " << endl;
        cout << "Deviation_angle:NULL" << endl;
    }

    if (flag_direction == 1) {
        cout << "Turn: Right" << endl;
    }
    else if (flag_direction == -1) {
        cout << "Turn: Left" << endl;
    }
    else {
        cout << "Turn: Forward" << endl;
    }
    cout << " " << endl;
    cout << "vel_track_x : " << vel_track[0] << " [m/s] " << endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] " << endl;
    cout << "vel_track_total : " << vel_track_total << " [m/s] " << endl;
    cout << "vel_collision_x : " << vel_collision_ENU[0] << " [m/s] " << endl;
    cout << "vel_collision_y : " << vel_collision_ENU[1] << " [m/s] " << endl;
    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] " << endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] " << endl;
    cout << "vel_sp_total : " << vel_sp_total << " [m/s] " << endl;
}

void printf_param()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "point1_x : " << point1_x << endl;
    cout << "point1_y : " << point1_y << endl;

    cout << "point2_x : " << point2_x << endl;
    cout << "point2_y : " << point2_y << endl;

    cout << "point3_x : " << point3_x << endl;
    cout << "point3_y : " << point3_y << endl;

    cout << "point4_x : " << point4_x << endl;
    cout << "point4_y : " << point4_y << endl;

    cout << "code1_x : " << code1_x << endl;
    cout << "code1_y : " << code1_y << endl;

    cout << "code2_x : " << code2_x << endl;
    cout << "code2_y : " << code2_y << endl;

    cout << "code3_x : " << code3_x << endl;
    cout << "code3_y : " << code3_y << endl;

    cout << "R_outside : " << R_outside << endl;
    cout << "R_inside : " << R_inside << endl;

    cout << "p_xy : " << p_xy << endl;
    cout << "vel_track_max : " << vel_track_max << endl;

    cout << "p_R : " << p_R << endl;
    cout << "p_r : " << p_r << endl;

    cout << "vel_collision_max : " << vel_collision_max << endl;
    cout << "nearly_stop : " << nearly_stop << endl;

    cout << "vel_sp_max : " << vel_sp_max << endl;
    cout << "range_min : " << range_min << endl;
    cout << "range_max : " << range_max << endl;
    cout << "fly_height: " << fly_height << endl;

    cout << "takeoff_time : " << takeoff_time << endl;
    cout << "hover_time: " << hover_time << endl;

    cout << "turn_angle1 : " << turn_angle1 << endl;
    cout << "turn_angle2 : " << turn_angle2 << endl;
    cout << "turn_angle3 : " << turn_angle3 << endl;
    cout << "turn_angle4 : " << turn_angle4 << endl;
    cout << "yaw_tolerance : " << yaw_tolerance << endl;
    cout << "turn_sp : " << turn_sp << endl;
    cout << "angle_judge : " << angle_judge << endl;
    cout << "fx : " << fx << endl;
    cout << "fy : " << fy << endl;
    cout << "cx : " << cx << endl;
    cout << "cy : " << cy << endl;
}

int calculate_angle_target(float target_x, float target_y)
{
    // 无人机当前位置
    float drone_x = pos_drone.pose.position.x;
    float drone_y = pos_drone.pose.position.y;

    // 计算目标点与无人机当前位置的向量
    float dx = target_x - drone_x;
    float dy = target_y - drone_y;

    // 计算目标点相对于无人机的角度（弧度）
    float angle_target_rad = atan2(dy, dx);

    // 将角度从弧度转换为度
    double current_yaw = Euler_fcu(2);
    int angle_target_deg = (angle_target_rad - current_yaw) / 3.1415926 * 180.0;

    if (angle_target_deg > 0) {
        return angle_target_deg;
    }
    else {
        return angle_target_deg + 360;
    }
}

int fly_direction() {
    //两个角度都是0到359度
    angle_diff = angle_c - angle_target;

    if (angle_diff > 180) {
        angle_diff -= 360;
    }
    else if (angle_diff <= -180) {
        angle_diff += 360;
    }

    if (0 <= angle_diff && angle_diff < 90) {
        return 1;
    }
    else  if (-90 < angle_diff && angle_diff < 0) {
        return -1;
    }
    else {
        return 0;
    }
}

bool judge_inline()
{
    if (abs(angle_diff) <= angle_judge && vel_sp_body[0] < nearly_stop && vel_sp_body[1] < nearly_stop) {

        diff_now = angle_diff;

        if (diff_now * diff_last < 0) {                 //判断夹角是否一直在左右变化，即正对障碍物卡住
            flag_direction = 1;                         //强行右转
        }
        diff_last = diff_now;

        return true;
    }
    else return false;
}


void detect_nav() 
{
    if (detect_num) {
        // 计算检测框中心像素坐标
        float pic_center_x = (yolov5_box.xmin + yolov5_box.xmax) / 2;
        float pic_center_y = (yolov5_box.ymin + yolov5_box.ymax) / 2;
        // 转换为相对无人机的坐标（假设使用前向摄像头）
        float depth = Laser.ranges[0];  // 使用正前方激光数据
        float dx = depth;
        float dy = (pic_center_x - cx) * depth / fx;
        // 更新目标点（ENU坐标系）
        pic_target[0] = pos_drone.pose.position.x + dx;
        pic_target[1] = pos_drone.pose.position.y - dy; 

        detect_mode = Off;
    }
}

void land_center(float& target_x, float& target_y)
{
    if (detect_num) {

        // 计算检测框中心像素坐标
        float u = (yolov5_box.xmin + yolov5_box.xmax) / 2.0;
        float v = (yolov5_box.ymin + yolov5_box.ymax) / 2.0;

        // 归一化坐标
        float x_norm = (u - cx) / fx;
        float y_norm = (v - cy) / fy;

        // 地面坐标
        target_x = fly_height * x_norm;
        target_y = fly_height * y_norm;
    }
}
string detect_result() {
    auto max_it = max_element(
        qr_map.begin(), qr_map.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; }
    );
    return max_it->first;
}