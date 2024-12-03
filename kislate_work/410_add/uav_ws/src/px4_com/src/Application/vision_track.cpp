#include <ros/ros.h>

#include <iostream>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>

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

px4_command::command Command_now;
//---------------------------------------Vision---------------------------------------------
geometry_msgs::Pose pos_target;                                 //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
int flag_detected = 0;                                          // 是否检测到目标标志
//---------------------------------------Track-----------------------------------------------
float distance_thres;

float kpy_track;                                                //追踪比例系数
float track_max_vel_y;                                          //追踪最大速度
float track_thres_vel_y;                                          //追踪速度死区
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int count_vision_lost = 0;                                                          //视觉丢失计数器阈值
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();                                                                 //打印函数
void generate_com(int sub_mode, float state_desired[4]);
float satfunc(float data, float Max, float Thres);                                   //限幅函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    pos_target = *msg;

    if(pos_target.orientation.w == 0)
    {
        num_count_vision_lost++;
    }else if(pos_target.orientation.w == 1)
    {
        flag_detected = 1;
        num_count_vision_lost = 0;
    }

    if(num_count_vision_lost > count_vision_lost)
    {
        flag_detected = 0;
    }

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_track");
    ros::NodeHandle nh("~");
    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标
    ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::Pose>("/vision/target", 10, vision_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //追踪算法比例参数
    nh.param<float>("kpy_track", kpy_track, 1.0);
    //追踪算法最大追踪速度,取决与使用场景，室内或者第一次实验时，建议选小一点
    nh.param<float>("track_max_vel_y", track_max_vel_y, 0.5);
    //追踪算法速度死区
    nh.param<float>("track_thres_vel_y", track_thres_vel_y, 0.02);
    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("count_vision_lost", count_vision_lost, 20);
    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //打印现实检查参数
    printf_param();

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
    }
    else return -1;

    int start_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check start，1 for go on， else for quit: "<<endl;
    cin >> start_flag;
    if(check_flag != 1) return -1;

    int comid = 0;
    while(ros::ok())
    {
        //回调
        ros::spinOnce();
        printf_result();

        //首先计算距离期望位置距离
        float distance;
        distance = sqrt( pos_target.position.y * pos_target.position.y);
        cout << "distance : "<< distance << endl;
        //如果 视觉丢失目标 或者 当前与目标距离小于距离阈值
        //发送悬停指令
        if(flag_detected == 0 || (distance < distance_thres))
        {
            Command_now.command = Hold;
        }
        //如果捕捉到目标
        else
        {
            //追踪是在机体系下完成
            Command_now.command = Move_Body;
            Command_now.sub_mode = 2;   //xy velocity z position
            Command_now.comid = comid;
            comid++;
            //X-orientation is locked
            Command_now.vel_sp[0] =  0;
            Command_now.vel_sp[1] =  - kpy_track * pos_target.position.y;
            //Height is locked.
            Command_now.pos_sp[2] =  0;
            //目前航向角锁定
            Command_now.yaw_sp = 0;
            //速度限幅
            Command_now.vel_sp[1] = satfunc(Command_now.vel_sp[1], track_max_vel_y, track_thres_vel_y);

            //如果期望速度为0,则直接执行悬停指令
            if(Command_now.vel_sp[1] == 0)
            {
                Command_now.command = Hold;
            }
        }
        command_pub.publish(Command_now);
        rate.sleep();
    }
    return 0;
}

void generate_com(int sub_mode, float state_desired[4])
{
    static int comid = 1;
    Command_now.sub_mode = sub_mode;

//# sub_mode 2-bit value:
//# 0 for position, 1 for vel, 1st for xy, 2nd for z.
//#                   xy position     xy velocity
//# z position       	0b00(0)       0b10(2)
//# z velocity		0b01(1)       0b11(3)

    if((sub_mode & 0b10) == 0) //xy channel
    {
        Command_now.pos_sp[0] = state_desired[0];
        Command_now.pos_sp[1] = state_desired[1];
    }
    else
    {
        Command_now.vel_sp[0] = state_desired[0];
        Command_now.vel_sp[1] = state_desired[1];
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_now.pos_sp[2] = state_desired[2];
    }
    else
    {
        Command_now.vel_sp[2] = state_desired[2];
    }

    Command_now.yaw_sp = state_desired[3];
    Command_now.comid = comid;
    comid++;
}


//饱和函数
float satfunc(float data, float Max, float Thres)
{
    if (abs(data)<Thres)
    {
        return 0;
    }
    else if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

void printf_result()
{
    cout.setf(ios::fixed);
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "flag_detected: " <<  flag_detected <<endl;
    cout << "num_count_vision_lost: " <<  num_count_vision_lost <<endl;

    cout << "pos_target: [X Y Z] : " << " " << pos_target.position.x  << " [m] "<< pos_target.position.y  <<" [m] "<< pos_target.position.z <<" [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Command: " << Command_now.vel_sp[0] << " [m/s] "<< Command_now.vel_sp[1] << " [m/s] "<< Command_now.vel_sp[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "kpy_track : "<< kpy_track << endl;
    cout << "track_max_vel_y : "<< track_max_vel_y << endl;
    cout << "track_thres_vel_y : "<< track_thres_vel_y << endl;
    cout << "count_vision_lost : "<< count_vision_lost << endl;
}


















