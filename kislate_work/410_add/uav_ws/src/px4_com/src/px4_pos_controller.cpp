#include <ros/ros.h>

#include <command_to_mavros.h>
#include <px4_command/command.h>
#include <pos_controller_PID.h>

//*************************//dyxtest
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//************************//

#include <Eigen/Eigen>

using namespace std;

using namespace namespace_command_to_mavros;
using namespace namespace_PID;

//自定义的Command变量
//相应的命令分别为 移动(惯性系ENU)，移动(机体系)，悬停，降落，上锁，紧急降落，待机
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

Eigen::Vector3d pos_sp(0,0,0);
Eigen::Vector3d vel_sp(0,0,0);
double yaw_sp = 0;
Eigen::Vector3d accel_sp(0,0,0);

//Command Now [from upper node]
px4_command::command Command_Now;                      //无人机当前执行命令

//Command Last [from upper node]
px4_command::command Command_Last;                     //无人机上一条执行命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_ros_time(ros::Time begin);
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const px4_command::command::ConstPtr& msg)
{
    Command_Now = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_controller");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<px4_command::command>("/px4/command", 10, Command_cb);

    ////dyxtest
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(50.0);

    command_to_mavros pos_controller;

    pos_controller.printf_param();

    pos_controller.show_geo_fence();

    pos_controller_PID pos_controller_pid;
    pos_controller_pid.printf_param();

    // 等待和飞控的连接
    while(ros::ok() && pos_controller.current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    // 连接成功
    ROS_INFO("Connected!!");

   //dyxtest
   geometry_msgs::PoseStamped pose;
   pose.pose.position.x = 0;
   pose.pose.position.y = 0;
   pose.pose.position.z = 0;

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<50;i++)
    {
        //dyxtest
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    pos_controller.set_takeoff_position();

    //初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令

    Command_Now.comid = 0;
    Command_Now.command = Idle;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = get_ros_time(begin_time);
    float dt = 0;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        // 当前时间
        float cur_time = get_ros_time(begin_time);
        dt = cur_time  - last_time;

        dt = constrain_function2(dt, 0.01, 0.03);

        last_time = cur_time;

        //Check for geo fence: If drone is out of the geo fence, it will land now.
        pos_controller.check_failsafe();

        //Printf the drone state
        //pos_controller.prinft_drone_state2(cur_time);

        //Printf the command state
        //prinft_command_state();

        //无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.command == Land)
        {
            Command_Now.command = Land;
        }

        switch (Command_Now.command)
        {
        case Move_ENU:
            pos_sp = Eigen::Vector3d(Command_Now.pos_sp[0],Command_Now.pos_sp[1],Command_Now.pos_sp[2]);
            vel_sp = Eigen::Vector3d(Command_Now.vel_sp[0],Command_Now.vel_sp[1],Command_Now.vel_sp[2]);
            accel_sp = pos_controller_pid.pos_controller(pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu, pos_sp, vel_sp, Command_Now.sub_mode, dt);

            pos_controller.send_accel_setpoint(accel_sp, Command_Now.yaw_sp);

            break;

        case Move_Body:
            //只有在comid增加时才会进入解算
            if( Command_Now.comid  >  Command_Last.comid )
            {
                //xy velocity mode
                if( Command_Now.sub_mode & 0b10 )
                {
                    float d_vel_body[2] = {Command_Now.vel_sp[0], Command_Now.vel_sp[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

                    rotation_yaw(pos_controller.Euler_fcu[2], d_vel_body, d_vel_enu);
                    vel_sp[0] = d_vel_enu[0];
                    vel_sp[1] = d_vel_enu[1];
                }
                //xy position mode
                else
                {
                    float d_pos_body[2] = {Command_Now.pos_sp[0], Command_Now.pos_sp[1]};         //the desired xy position in Body Frame
                    float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                    rotation_yaw(pos_controller.Euler_fcu[2], d_pos_body, d_pos_enu);

                    pos_sp[0] = pos_controller.pos_drone_fcu[0] + d_pos_enu[0];
                    pos_sp[1] = pos_controller.pos_drone_fcu[1] + d_pos_enu[1];
                }

                //z velocity mode
                if( Command_Now.sub_mode & 0b01 )
                {
                    vel_sp[2] = Command_Now.vel_sp[2];
                }
                //z posiiton mode
                {
                    pos_sp[2] = pos_controller.pos_drone_fcu[2] + Command_Now.pos_sp[2];
                }

                yaw_sp = pos_controller.Euler_fcu[2]* 180/M_PI + Command_Now.yaw_sp;

            }

            accel_sp = pos_controller_pid.pos_controller(pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu, pos_sp, vel_sp, Command_Now.sub_mode, dt);

            pos_controller.send_accel_setpoint(accel_sp, yaw_sp);

            break;

        case Hold:
            if (Command_Last.command != Hold)
            {
                pos_controller.Hold_position = Eigen::Vector3d(pos_controller.pos_drone_fcu[0],pos_controller.pos_drone_fcu[1],pos_controller.pos_drone_fcu[2]);
                pos_controller.Hold_yaw = pos_controller.Euler_fcu[2]* 180/M_PI;
                pos_sp = pos_controller.Hold_position;
                yaw_sp = pos_controller.Hold_yaw;
            }
            accel_sp = pos_controller_pid.pos_controller(pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu, pos_controller.Hold_position, vel_sp, Command_Now.sub_mode, dt);

            pos_controller.send_accel_setpoint(accel_sp, pos_controller.Hold_yaw);

            break;


        case Land:
            if (Command_Last.command != Land)
            {
                pos_sp = Eigen::Vector3d(pos_controller.pos_drone_fcu[0],pos_controller.pos_drone_fcu[1],pos_controller.Takeoff_position[2]);
                yaw_sp = pos_controller.Euler_fcu[2]* 180/M_PI;
            }

            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if(abs(pos_controller.pos_drone_fcu[2] - pos_controller.Takeoff_position[2]) < (0.1))
            {
                if(pos_controller.current_state.mode == "OFFBOARD")
                {
                    pos_controller.mode_cmd.request.custom_mode = "MANUAL";
                    pos_controller.set_mode_client.call(pos_controller.mode_cmd);
                }

                if(pos_controller.current_state.armed)
                {
                    pos_controller.arm_cmd.request.value = false;
                    pos_controller.arming_client.call(pos_controller.arm_cmd);

                }

                if (pos_controller.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {  
                accel_sp = pos_controller_pid.pos_controller(pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu, pos_sp, vel_sp, 0b00, dt);
                pos_controller.send_accel_setpoint(accel_sp, yaw_sp);
            }

            break;

        case Disarm:
            if(pos_controller.current_state.mode == "OFFBOARD")
            {
                pos_controller.mode_cmd.request.custom_mode = "MANUAL";
                pos_controller.set_mode_client.call(pos_controller.mode_cmd);
            }
            if(pos_controller.current_state.armed)
            {
                pos_controller.arm_cmd.request.value = false;
                pos_controller.arming_client.call(pos_controller.arm_cmd);

            }
            if (pos_controller.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }
            break;
        
        case Arm://上电操作。
            if(pos_controller.current_state.mode != "OFFBOARD")//这个是使得飞机转为电脑控制/
            {
                cout<<"Please switch to OFFBOARD mode!"<<endl;
                break;
            }
            if(!pos_controller.current_state.armed)
            {
                 pos_controller.arm_cmd.request.value = true;//request是对飞控的请求
                 //设置了以后在下面的call发送
                 pos_controller.arming_client.call(pos_controller.arm_cmd);//arming_client 是一个 ROS 服务客户端，用于向飞控系统的
                 // arming 服务发送命令。这通常是一个客户端接口，用来与飞控系统的服务进行通信。
            }
            if (pos_controller.arm_cmd.response.success)//response是飞控对请求的响应。
            {
                cout<<"Arm successfully!"<<endl;
               
            }
            break;
        
        case Failsafe_land:
            break;

        case Idle:
            pos_controller.idle();
            break;

        case Takeoff:
            //pos_controller.mode_cmd.request.custom_mode = "OFFBOARD";
            //pos_controller.set_mode_client.call(pos_controller.mode_cmd);


            pos_sp = Eigen::Vector3d(pos_controller.Takeoff_position[0],pos_controller.Takeoff_position[1],pos_controller.Takeoff_position[2]+pos_controller.Takeoff_height);
            vel_sp = Eigen::Vector3d(0.0,0.0,0.0);
            accel_sp = pos_controller_pid.pos_controller(pos_controller.pos_drone_fcu, pos_controller.vel_drone_fcu, pos_sp, vel_sp, Command_Now.sub_mode, dt);

            pos_controller.send_accel_setpoint(accel_sp, Command_Now.yaw_sp);//

            break;
        }

        Command_Last = Command_Now;

        rate.sleep();//rate.sleep() 是与 ros::Rate搭配的，可以让循环以固定时间运行
    }
    return 0;
}

// 【获取当前时间函数】 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    int sub_mode;
    sub_mode = Command_Now.sub_mode;

    switch(Command_Now.command)
    {
    case Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;

        if((sub_mode & 0b10) == 0) //xy channel
        {
            cout << "Submode: xy position control "<<endl;
            cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.pos_sp[1] << " [ m ]"<<endl;
        }
        else{
            cout << "Submode: xy velocity control "<<endl;
            cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.vel_sp[1] << " [m/s]" <<endl;
        }

        if((sub_mode & 0b01) == 0) //z channel
        {
            cout << "Submode:  z position control "<<endl;
            cout << "Z_setpoint   : "<< Command_Now.pos_sp[2] << " [ m ]" << endl;
        }
        else
        {
            cout << "Submode:  z velocity control "<<endl;
            cout << "Z_setpoint   : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
        }

        cout << "Yaw_setpoint : "  << Command_Now.yaw_sp << " [deg] " <<endl;

        break;
    case Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;

        if((sub_mode & 0b10) == 0) //xy channel
        {
            cout << "Submode: xy position control "<<endl;
            cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.pos_sp[1] << " [ m ]"<<endl;
        }
        else{
            cout << "Submode: xy velocity control "<<endl;
            cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.vel_sp[1] << " [m/s]" <<endl;
        }

        if((sub_mode & 0b01) == 0) //z channel
        {
            cout << "Submode:  z position control "<<endl;
            cout << "Z_setpoint   : "<< Command_Now.pos_sp[2] << " [ m ]" << endl;
        }
        else
        {
            cout << "Submode:  z velocity control "<<endl;
            cout << "Z_setpoint   : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
        }

        cout << "Yaw_setpoint : "  << Command_Now.yaw_sp << " [deg] " <<endl;

        break;

    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        cout << "Hold Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << yaw_sp << " [deg] " <<endl;
        break;

    case Land:
        cout << "Command: [ Land ] " <<endl;
        cout << "Land Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << Command_Now.yaw_sp << " [deg] " <<endl;
        break;

    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;

    case Failsafe_land:
        cout << "Command: [ Failsafe_land ] " <<endl;
        break;

    case Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;

    case Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        cout << "Takeoff Position [X Y Z] : " << pos_sp[0] << " [ m ] "<< pos_sp[1]<<" [ m ] "<< pos_sp[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << Command_Now.yaw_sp << " [deg] " <<endl;
        break;
    }



}
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}
