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
#include <algorithm> // 添加此头文件以使用 std::sort

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
//--------------------------------------------识别--------------------------------------------------
// int flag_hold;                                                   //悬停标志

float fx=554.3827;                                               //相机内参
float fy=554.3827;
float cx=320;
float cy=240;
// float pic_target[2];                                             //模式4的图像中心ENU坐标
// float abs_distance1=10;                                          //为模式4中穿越2门与识别图像之间的过度而设置的最小距离值

int Class_Id_target = -1;
int Class_Id_now = -1;
int door_num = 0;// 或许有用
float door_center[2] = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};

float qr_target_x;                                //图片中心x坐标
float qr_target_y;                                //图片中心y坐标
float MIN_SCORE = 0.5;

float target_x_1;
float target_y_1;

float target_x_2;
float target_y_2;

float target_x_3;
float target_y_3;
//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float last_time = 0;
float fly_height;
float fly_forward;
float fly_turn = -90;
//--------------------------------------------算法相关--------------------------------------------------
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c,angle_c;                                       //最近障碍物距离 角度
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
float R_inside;                                                 //内圈半径
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>识别函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void confirm_ID(bool& flag_qr);
void find_ID();
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
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是世界坐标系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
struct doorfind {
    int start;
    int end;
    int length;
} line[180];
int linefind(float height[181]);
void doorfind();
int change(int i);
int key[4] = {-1, -1, -1, -1};
float door_find_location[2];
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;
    int count;    //count = 359
    count = Laser.ranges.size();

    //剔除inf的情况
    // for(int i = 0; i < count; i++)
    // {
    //     //判断是否为inf
    //     int a = isinf(Laser_tmp.ranges[i]);
    //     //如果为inf，则赋值上一角度的值
    //     if(a == 1)
    //     {
    //         if(i == 0)
    //         {
    //             Laser_tmp.ranges[i] = Laser_tmp.ranges[count-1];
    //         }
    //         else
    //         {
    //             Laser_tmp.ranges[i] = Laser_tmp.ranges[i-1];
    //         }
    //     }
    
    // }
    for(int i = 0; i < count; i++)
    {
           if(i+180>359) Laser.ranges[i]=Laser_tmp.ranges[i-180];
           else Laser.ranges[i]=Laser_tmp.ranges[i+180];
           //cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
    }
    //cout<<"//////////////"<<endl;
    //计算前后左右四向最小距离
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

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0); //dyx
    nh.param<float>("target_y", target_y, 0.0); //dyx
    nh.param<float>("R_inside", R_inside, 0.6);
    nh.param<int>("range_min", range_min, 0);
    nh.param<int>("range_max", range_max, 0);
    //nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);
    nh.param<float>("fly_height", fly_height, 0.5);
    nh.param<float>("fly_forward", fly_forward, 0.8);
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
    if(Take_off_flag == 1)
    {
        Command_now.command = Takeoff;
        command_pub.publish(Command_now);
        cout << "Point 0----->takeoff"<<endl;
    }
    else return -1;
    // int stay_flag;
    // cout<<"Whether choose to Stay? 1 for stay, 0 for quit"<<endl;
    // cin >> stay_flag;
    int comid = 0;
    int i = 0;
    sleep_time = sleep_time * 20;
    // while(abs(pos_drone.pose.position.z - fly_height) > 0.1)
    // {
    //     Command_now.command = Move_ENU;
    //     Command_now.sub_mode = 0;
    //     Command_now.pos_sp[0] = 0;
    //     Command_now.pos_sp[1] = 0;
    //     Command_now.pos_sp[2] = fly_height;
    //     Command_now.yaw_sp = 0;
    //     Command_now.comid = comid;
    //     comid++;
    //     command_pub.publish(Command_now);
    //     cout << "Point 0----->takeoff"<<endl;
    //     cout << "z = "<<pos_drone.pose.position.z<<endl;
    //     cout << "target = "<<fly_height<<endl;
    // }

    // while (i < sleep_time)
    // {
    //     Command_now.command = Move_ENU;
    //     Command_now.sub_mode = 0;
    //     Command_now.pos_sp[0] = fly_forward;
    //     Command_now.pos_sp[1] = 0;
    //     Command_now.pos_sp[2] = fly_height;
    //     Command_now.yaw_sp = 0;
    //     Command_now.comid = comid;
    //     comid++;
    //     command_pub.publish(Command_now);
    //     rate.sleep();
    //     cout << "Point 0.5----->stay"<<endl;
    //     cout << "z = "<<pos_drone.pose.position.z<<endl;
    //     cout << "target = "<<fly_height<<endl;
    //     i++;
    // }

    //check start collision_avoid
    int start_flag;
    cout<<"Whether choose to Start mission? 1 for start, 0 for quit"<<endl;
    cin >> start_flag;
    if(Take_off_flag != 1) return -1;

    //初值
    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;

    flag_land = 0;


    float abs_distance = 1e5;
    //第一步，前进0.5~0.8米
    while (abs_distance > 0.3) {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = fly_forward;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 1----->move forward" << endl;
        cout << "x = "<<pos_drone.pose.position.x<< endl;
        cout << "target = "<<fly_forward<< endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }
    //需要添加悬停
    i = 0;
    while (i < sleep_time)
    {
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = fly_forward;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 1.5----->stay"<<endl;
        cout << "i = "<<i<<endl;
        i++;
    }

	// int turn_flag;
	// cout<<"Whether choose to Start turn? 1 for start, 0 for quit"<<endl;
    // cin >> turn_flag;
    //第二步，转90度
    float turn_angle=0;
    while (abs(Euler_fcu[2] * 180.0/M_PI - fly_turn)>3){
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = fly_forward;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = turn_angle;
        turn_angle=turn_angle - 1.0 ;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "Point 2----->turn 90" << endl;
        cout << "yaw_angle  " << Euler_fcu[2] * 180.0/M_PI <<"  du"<<endl;
        cout << "target_angle  " << turn_angle <<"  du"<<endl;
    }

    int door_flag;
    cout<<"Whether choose to Start? 1 for start, 0 for quit"<<endl;
    cin >> door_flag;
//From 刘芯贝
//     int MAX_ATTEMPTS = 10;  // 最大尝试次数
//     int attempts = 0;
//     bool door_found = false;
//  // 循环调用 doorfind 函数
//     while (attempts < MAX_ATTEMPTS && !door_found) {
//         ros::spinOnce();  // 更新传感器数据
//         doorfind();
//         // 这里可以根据 doorfind 函数的结果判断是否找到门
//         // 假设当 door_find_location[0] 和 door_find_location[1] 不为 -1 时表示找到门
//         if (door_find_location[0] != -1 && door_find_location[1] != -1) {
//             door_found = true;
//             cout << "Door found after " << attempts + 1 << " attempts." << endl;
//         } else {
//             attempts++;
//             cout << "Door not found in attempt " << attempts << ". Retrying..." << endl;
//             rate.sleep();
//         }
//     }
//     if (!door_found) {
//         cout << "Failed to find the door after " << MAX_ATTEMPTS << " attempts." << endl;
//         return -1;
//     } 
    //第三步，穿门
    // ros::spinOnce();
    // doorfind();
    abs_distance = 1e5;
    while (abs_distance > 0.1){
        ros::spinOnce(); // Add this line to process callbacks
        doorfind();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = door_find_location[0];
        Command_now.pos_sp[1] = pos_drone.pose.position.y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = fly_turn ;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 3----->passing_door" << endl;
        cout << "passing_door_x" << endl;
        cout << "x = "<<pos_drone.pose.position.x<< endl;
        cout << "target_x= "<<door_find_location[0]<< endl;
        cout << "y = "<<pos_drone.pose.position.y<< endl;
        cout << "target_y= "<<door_find_location[1]<< endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }

    // int avoidance_flag;
    // cout<<"Whether choose to Start avoidance? 1 for start, 0 for quit"<<endl;
    // cin >> avoidance_flag;

    abs_distance = 1e5;
    while (abs_distance > 0.1){
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = door_find_location[0];
        Command_now.pos_sp[1] = door_find_location[1];
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = fly_turn ;
        Command_now.comid = comid;
        comid++;
        command_pub.publish(Command_now);
        rate.sleep();
        ros::spinOnce(); // Add this line to process callbacks
        cout << "passing_door" << endl;
        cout << "x = "<<pos_drone.pose.position.x<< endl;
        cout << "target_x= "<<door_find_location[0]<< endl;
        cout << "y = "<<pos_drone.pose.position.y<< endl;
        cout << "target_y= "<<door_find_location[1]<< endl;
        abs_distance = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
    }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    // 前往二维码识别区
    float qr_abs_distance = 1e5;
    bool flag_qr = false;
    bool flag_target_1 = false;
    while (qr_abs_distance > 0.2)
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        cone_avoidance(qr_target_x,qr_target_y);

        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = fly_turn ;
        float abs_distance;
        qr_abs_distance = sqrt((pos_drone.pose.position.x - qr_target_x) * (pos_drone.pose.position.x - qr_target_x) + (pos_drone.pose.position.y - qr_target_y) * (pos_drone.pose.position.y - qr_target_y));
        command_pub.publish(Command_now);
        //打印
        printf();
        rate.sleep();
        cout << "Point 4----->move to qr" << endl;
        cout << "qr_target_x = "<<qr_target_x<< "  ";
        cout << "qr_target_y = "<<qr_target_y<< endl;
        cout << "x = "<<pos_drone.pose.position.x<< "  ";
        cout << "y = "<<pos_drone.pose.position.y<< endl;
    }


    // 悬停, 识别二维码
    i = 0;
    while (i < sleep_time)
    {
        ros::spinOnce(); 
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = qr_target_x;
        Command_now.pos_sp[1] = qr_target_y;
        Command_now.pos_sp[2] = fly_height;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;
        if(!flag_qr){
            cout << "QR code not detected" << endl;
            confirm_ID(flag_qr);
        }
        else{
            cout << "QR code detected " << " ID: " << Class_Id_target << endl;
        }
        command_pub.publish(Command_now);
        rate.sleep();
        cout << "Point 4.5----->Detecting qrcode"<<endl;
        cout << "i = "<<i<<endl;
        i++;
    }

    // 加入没有识别到

    // 识别二维码后，前往目标点
    // 方案一:分别前往三个目标点比对
    // 方案二: 巡航识别,将新坐标点入列,逐个试探

    int case_num = 0;
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        cone_avoidance(target_x,target_y);

        switch(case_num){
            case 0:
                target_x = target_x_1;
                target_y = target_y_1;
                break;
            case 1:
                target_x = target_x_2;
                target_y = target_y_2;
                break;
            case 2:
                target_x = target_x_3;
                target_y = target_y_3;
                break;
            default:
                break;
        }
        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = fly_turn ;
        float abs_distance;
        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
        if(abs_distance < 0.2 )// 感觉不妥
        {
            i = 0;
            while(i < sleep_time)
            {
                Command_now.command = Hold;
                command_pub.publish(Command_now);
                rate.sleep();
                i++;
            }
        }
        case_num++;
        if(flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);
        //打印
        printf();
        rate.sleep();
    }
    /*
        while：加入巡航，识别二维码，降落等功能
    */
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
    cout<<"fly heigh: "<<fly_height<<endl;
    cout<<"fly forward: "<<fly_forward<<endl;
    cout<<"fly turn: "<<fly_turn<<endl;
    cout<<"sleep_time "<<sleep_time<<endl;
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
int change(int i){
    if (i<=90) return 90-i;
    else return 450-i;
}
int linefind(float height[181]) {
    float minus[180];
    for (int i = 0; i < 180; i++) {
        minus[i] = abs(height[i] - height[i + 1]);   // 第 i 个点和第 i+1 个点的高度差
        line[i].length = 1;
    }
    line[0].start = 0;
    int key = 0;
    for (int i = 0; i < 180; i++) {
        if (minus[i] < 0.2 && !isinf(height[i]) ) {
            line[key].length++;
            line[key].end = i + 1;
        } else {
            line[key].end = i;
            key++;
            line[key].start = i + 1;
        }
    }
    return key + 1; // 返回找到的线段数量
}
void doorfind(){
    float length[181];
    float height[181];
    for(int i=0;i<=180;i++){
        length[i]=Laser.ranges[change(i)];
        height[i]=length[i]*sin(i * M_PI / 180);
    }
    int num_lines = linefind(height);
    int max1 = -1, max2 = -1;
    int max1_index = -1, max2_index = -1;
    for (int i = 0; i < num_lines; i++) {
        if (line[i].length > max1) {
            max2 = max1;
            max2_index = max1_index;
            max1 = line[i].length;
            max1_index = i;
        } else if (line[i].length > max2) {
            max2 = line[i].length;
            max2_index = i;
        }
    }
    if (max1_index != -1) {
        key[0] = line[max1_index].start;
        key[1] = line[max1_index].end;
    }
    if (max2_index != -1) {
        key[2] = line[max2_index].start;
        key[3] = line[max2_index].end;
    }
    // 对 key 数组进行排序
    std::sort(key, key + 4);
    // 取第二大和第三大的元素
    int second_largest = key[2];
    int third_largest = key[1];
    // 计算平均数
    int drone_angle = (second_largest + third_largest) / 2.0;
    drone_angle = change(drone_angle);
    float world_angle = drone_angle + Euler_fcu[2] * 180.0 / M_PI;  // 将机体系下的角度转换为世界坐标系下的角度
    normalize_angle(&world_angle);   // 将角度调整到 -180 到 180 度范围内
    float y_length = (height[second_largest] + height[third_largest]) / 2.0;
    float x_length = y_length * tan(drone_angle * M_PI / 180);
    door_find_location[0] = pos_drone.pose.position.x + x_length;
    door_find_location[1] = pos_drone.pose.position.y - y_length - 0.2;
}





// void find_ID(){
//     // 如果是目标ID，找到其中心
//     flag_land = 1;
//     if(darknet_boxes.bounding_boxes.size() > 0){
//         for(int i = 0; i < darknet_boxes.bounding_boxes.size(); i++){
//             if(darknet_boxes.bounding_boxes[i].Class_Id == Class_Id_target){
//                 qr_cx = (darknet_boxes.bounding_boxes[i].xmin + darknet_boxes.bounding_boxes[i].xmax) / 2;
//                 qr_cy = (darknet_boxes.bounding_boxes[i].ymin + darknet_boxes.bounding_boxes[i].ymax) / 2;
//             }// 这里只是像素坐标中心
//         }
//     }   
//     // 加入坐标转换
//     // 误差处理，实时更新坐标
// }
