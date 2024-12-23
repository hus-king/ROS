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
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float last_time = 0;
float fly_height;
//--------------------------------------------算法相关--------------------------------------------------
float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c,angle_c;                                       //最近障碍物距离 角度
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_max;                                               //总速度限幅
px4_command::command Command_now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
void printf();                                                                       //打印函数
void printf_param();   
struct doorfind {
    int start;
    int end;
    int length;
} line[180];
int linefind(float height[181]);
void doorfind();
int change(int i);
int key[4] = {-1, -1, -1, -1};
void normalize_angle(float *angle);
float door_find_location[2];                                                              //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;
    Laser = *scan;
    int count;
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
    }
    cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);
    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);
    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0); //dyx
    nh.param<float>("target_y", target_y, 0.0); //dyx

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
    nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);
    //打印现实检查参数
    printf_param();

    int start_flag;
    cout<<"Whether choose to Start test? 1 for start, 0 for quit"<<endl;
    cin >> start_flag;


    int comid = 1;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        cout<<"ok"<<endl;
        doorfind();
        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  0;
        Command_now.vel_sp[1] =  0;  //ENU frame
        Command_now.pos_sp[2] =  0;
        Command_now.yaw_sp = 0 ;
        command_pub.publish(Command_now);
        rate.sleep();
    }
    return 0;
}

//计算前后左右四向最小距离
void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    
    for (int i = range_min*2; i <= range_max*2; i++)
    {
        if(Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i;
        }
    }
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;

    cout << "R_outside : "<< R_outside << endl;
    cout << "R_inside : "<< R_inside << endl;

    cout << "p_xy : "<< p_xy << endl;
    cout << "vel_track_max : "<< vel_track_max << endl;

    cout << "p_R : "<< p_R << endl;
    cout << "p_r : "<< p_r << endl;

    cout << "vel_collision_max : "<< vel_collision_max << endl;

    cout << "vel_sp_max : "<< vel_sp_max << endl;
    cout << "range_min : "<< range_min << endl;
    cout << "range_max : "<< range_max << endl;
    cout<<"fly heigh: "<<fly_height<<endl;
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
        cout<<i<<" : "<<height[i]<<endl;
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
    cout << "Longest lines:" << endl;
    cout << "Line 1: Start = " << key[0] << ", End = " << key[1] << endl;
    cout << "Line 2: Start = " << key[2] << ", End = " << key[3] << endl;
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
    cout << "world_angle : " << world_angle << " [du] "<<endl;
    float x_length = (height[second_largest] + height[third_largest]) / 2.0;
    float y_length = x_length * tan(drone_angle * M_PI / 180);
    door_find_location[0] = pos_drone.pose.position.x + x_length+0.2;
    door_find_location[1] = pos_drone.pose.position.y + y_length;
    cout << "door_find_location : " << door_find_location[0] << " [m] "<< door_find_location[1] << " [m] "<<endl;
}