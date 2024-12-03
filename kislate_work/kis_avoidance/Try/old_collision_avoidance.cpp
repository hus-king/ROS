#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>//无人机飞控行为
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
//以下都是全局变量，在函数中自动改变并且更新
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
float vel_collision_EMU[2];//世界坐标系下的
float vel_collision_max;                                        //躲避障碍部分速度限幅
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度//setpoint--sp
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_max;                                               //总速度限幅
px4_command::command Command_now;                               //发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();//声明一个距离最小函数的求值。
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
void collision_avoidance(float target_x,float target_y);
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
/*激光雷达主要包含的字段。
namespace sensor_msgs
{
    struct LaserScan
    {
        std_msgs::Header header;    // 消息头，包含时间戳、坐标系等信息
        float angle_min;            // 激光扫描的最小角度 (单位：弧度)
        float angle_max;            // 激光扫描的最大角度 (单位：弧度)
        float angle_increment;      // 每个激光点的角度增量 (单位：弧度)
        float time_increment;       // 激光扫描每个点的时间增量（单位：秒）
        float scan_time;            // 一次完整扫描的时间（单位：秒）
        float range_min;            // 激光雷达测量的最小距离（单位：米）
        float range_max;            // 激光雷达测量的最大距离（单位：米）
        std::vector<float> ranges; // 每个激光点的距离值（单位：米）
        这里的距离值是不考虑半径的。
        std::vector<float> intensities;  // 每个激光点的强度值（单位：无量纲，取决于激光雷达的类型）
    };
}

*/
void rotation_yaw(float yaw_angle, float input[2], float output[2])//input是机体系,output是世界坐标系，yaw_angle是当前偏航角
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}//（cosx,sinx）T,(-sinx,cosx)T;
//乘了个矩阵。
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
//剔除inf的情况
    //inf 就是没有测到障碍物，或者没有得到有效反射/
    /*优化建议：
    平滑处理：对于连续多个 inf 值，可以考虑进行平滑处理，使用多个邻近的数据值来平均填充 inf，
    而不是直接用前一个数据。这样可以避免单次替代对数据的突然跳变造成影响。
    检测极大值：除了 inf，有时激光雷达的某些测量值可能会非常大（例如超出了有效距离范围）
    ，这些值也需要特殊处理，可能通过设定一个最大距离阈值进行过滤。*/
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)//每次接受到雷达传回来的数据会触发。
//sensor_msgs::LaserScan::ConstPtr& scan： 这是一个指向常量的指针，表示接收到的激光雷达数据。该数据包含了激光雷达在各个角度上的距离测量值。
{
    sensor_msgs::LaserScan Laser_tmp;
    Laser_tmp = *scan;//这个也是一个sensor_msgs::LaserScan 类型的一个变量，它被用来存储一个从 scan 消息接收到的副本。
    Laser = *scan;//sensor_msgs::LaserScan Laser; 这处代码在这里定义；
    int count;    //count = 359//这个359不知道什么意思。在这里laser是全局的，laser_temp是临时的，用于后续计算决策。
    count = Laser.ranges.size();//在这里获取数组的大小，确保不会越界。就是上面那个359.
    //我怀疑这里的count不是359.可能是719.
    //因为360和0是重合的（

    /*count = Laser.ranges.size(); 这一行代码的作用是获取激光雷达数据中距离测量值的数量。具体来说，它是获取激光雷达扫描数据中的 ranges 数组的长度。

    详细解释：
    在 ROS 中，激光雷达的数据通常使用 sensor_msgs::LaserScan 
    消息类型来表示。这个消息类型的关键字段之一是 ranges，
    它是一个 std::vector<float> 类型的数组，
    表示激光雷达在不同角度（通常是水平角度）上的距离测量值。

    ranges 数组：
    ranges 是一个包含激光雷达在各个方向上的距离测量值的数组。
    每个元素表示雷达在某一特定角度的测量距离。
    数组的长度通常与激光雷达的分辨率或扫描的角度数量有关。
    例如，如果激光雷达扫描了360度，并且每度有一个测量值，那么 ranges 数组的长度就是 360。*/

    
    for(int i = 0; i < count; i++)
    {
        //判断是否为inf
        int a = isinf(Laser_tmp.ranges[i]);//C++中的标准库函数判断数值是否为“无穷大”或者“无穷小”。
        //括号中的是laser在第i个角度的测量值
        //如果为inf，则赋值上一角度的值
        if(a == 1)//笑死，不知道这个东西是不是可以直接if（Laser_tmp.ranges[i]）
        //gpt说这个东西可能会影响后续处理，因为isinf返回的是一个float的值。
        {
            if(i == 0)
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[count-1];
            }
            else
            {
                Laser_tmp.ranges[i] = Laser_tmp.ranges[i-1];
            }
        }
        /*处理 inf 的原因：
        避免影响其他数据计算： 如果激光雷达的某个角度返回了 inf，
        它将极大地影响后续的计算。如果不进行处理，
        这个 inf 值可能会影响你计算最小距离、障碍物的位置等重要数据，导致算法无法正常工作。

        确保平滑性： 在实际使用中，如果一个角度的测量值为 inf，
        意味着在这个角度上没有障碍物被检测到（例如距离太远）。
        直接用 inf 会导致后续的计算出现异常，比如速度控制或者路径规划时，
        可能会错误地认为某个角度没有障碍物。如果你简单地将 inf 替换为前一个有效的值，
        能够平滑地过渡到下一个有效的点，从而保持算法的稳定性。

        数据的连续性： 激光雷达通常是进行旋转扫描的，
        数据是按照扫描角度逐步获得的。在激光雷达扫描的过程中，某些角度可能因为环境因素
        （例如视野过远、无障碍物、天气等原因）没有障碍物被检测到。为了保持数据的连续性，
        通常会用相邻的有效数据来填补 inf，这可以使得数据更加平滑，并避免算法受到干扰。*/
    
        }
    for(int i = 0; i < count; i++)
    {
           if(i+360>719) Laser.ranges[i]=Laser_tmp.ranges[i-360];//？我不理解这个算法。i为什么会大于719，不是说count是359吗。
           else Laser.ranges[i]=Laser_tmp.ranges[i+360];
           //cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
    }
    //cout<<"//////////////"<<endl;
    //计算前后左右四向最小距离
    cal_min_distance();//具体操作见300行以后
    /*void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];
    angle_c = 0;
    
    for (int i = range_min*2; i <= range_max*2; i++)
    {
        if(Laser.ranges[i] < distance_c)
        {
            distance_c = Laser.ranges[i];
            angle_c = i/2;
            //angle_c = i;
        }
    }
}*/
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)//ConstPtr一个指向消息类型的指针。
/*PoseStamped 消息包含了位置信息（position）和姿态信息（orientation）。
PoseStamped 中的数据类型是 geometry_msgs::Pose，
而 PoseStamped 本身还包含了时间戳和参考框架等信息。*/
{
    pos_drone = *msg;//geometry_msgs::PoseStamped pos_drone; ‘*’来解指针。
    //msgs本身也是个指针
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    //Eigen 是一个高效的 C++ 数学库，主要用于线性代数、矩阵操作和数值计算。
    //Quaaterniond 就是Eigen中的四元数类型
    /*x,y,z,w是四元数组的四个元素*/
    //不是很了解


    /*namespace geometry_msgs
{
  struct Pose
  {
    Point position;         // 机器人的位置
    Quaternion orientation; // 机器人的方向（旋转）
  };

  struct PoseStamped
  {
    std_msgs::Header header;  // 时间戳和坐标系信息
    Pose pose;                // 机器人位置和方向
  };
}*/


    q_fcu = q_fcu_enu;//Eigen::Quaterniond q_fcu;这个也是一个全局变量
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);//Eigen::Vector3d Euler_fcu;这是一个三维向量
    /*#include <Eigen/Dense>
      #include <Eigen/Geometry>*/
    //#include <math_utils.h>这个px4的函数库里面有这个将四元数转换为欧拉角的函数
    //（滚转、俯仰和航向）从前至后，存在旋转顺序，顺序在前的轴旋转会带动后面轴的旋转，而反之不然（动态坐标下）。
    //欧拉角存在万向死锁现象
    //四元数去看3b1b
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");//初始化节点。
    /*通过 roslaunch 传递参数：

    使用 roslaunch 启动时，你也可以在命令行中传递参数。
    在 Launch 文件中，通常会使用 <arg> 标签来定义可传递的参数。
    假设你有一个 collision_avoidance.launch 文件，如下所示：
    <launch>
        <arg name="rate" default="20" />
        <param name="rate" value="$(arg rate)" />
        <node name="collision_avoidance" pkg="my_package" type="collision_avoidance" />
    </launch>
    */
   //roslaunch my_package collision_avoidance.launch rate:=50 命令行操作
    ros::NodeHandle nh("~");//私有命名空间
    // 频率 [20Hz]
    ros::Rate rate(20.0);//这个就是发布频率
    //【订阅】Lidar数据//ros::Subscriber订阅器
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
    //acsn是名字，1000是订阅队列大小，防止消息过多过快，最后那个是回调函数，这个回调函数就是上面那玩意
    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    //表示消息队列的大小为 10。如果发布频率过高，且队列已满，新的消息将会被丢弃。

    //读取参数表中的参数，这些东西都是从yml中找
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
    nh.param<int>("range_max", range_max, 0.0);//相对路径
    nh.getParam("/px4_pos_controller/Takeoff_height",fly_height);//在ros平台的这个.....中找到这个参数‘
    //绝对路径
    //ros::NodeHandle，都是这个类的成员函数。
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
    if(Arm_flag == 1)//上电
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
    }//起飞
    else return -1;

    //check start collision_avoid
    int start_flag;
    cout<<"Whether choose to Start mission? 1 for start, 0 for quit"<<endl;
    cin >> start_flag;
    if(Take_off_flag != 1) return -1;//再次检查

    //初值
    vel_track[0]= 0;
    vel_track[1]= 0;

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    vel_sp_ENU[0]= 0;
    vel_sp_ENU[1]= 0;

    flag_land = 0;

    int comid = 1;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        collision_avoidance(target_x,target_y);//这里就是避障程序

        Command_now.command = Move_ENU;     //机体系下移动
        Command_now.comid = comid;
        comid++;//用于区分不同的命令，就是个id
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_ENU[0];  //这堆东西在px4的头文件里都有
        Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
        Command_now.pos_sp[2] =  fly_height;
        Command_now.yaw_sp = 0 ;

        float abs_distance;
        abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
        if(abs_distance < 0.3 || flag_land == 1)
        {
            Command_now.command = 3;     //Land
            flag_land = 1;
        }
        if(flag_land == 1) Command_now.command = Land;
        command_pub.publish(Command_now);//把以上关于command的所有命令都发送上去，包括速度之类的，还有一个主命令，command
        //打印
        printf();
        rate.sleep();//和rate搭配，就是命令刷新速率。
    }
    return 0;
}

//计算前后左右四向最小距离
void cal_min_distance()
{
    distance_c = Laser.ranges[range_min];//初始为雷达中探测到的最小距离
    angle_c = 0;//初始最小距离角度为0
    
    for (int i = range_min*2; i <= range_max*2; i++)//*2的意思难道是一个i只有0.5吗。一共有720个点？
    //i在这里可以看作是角度，因为整个数组其实就是将雷达扫到的所有角度平分
    {
        if(Laser.ranges[i] < distance_c)//如果当前最小距离小于最小距离中的最小距离，更新为记录的最小距离
        {
            distance_c = Laser.ranges[i];
            angle_c = i/2;//这个可能是因为一个i就只是0.5，精度比较高。
            //angle_c = i;// 如果不需要转换为角度（保持原代码），可以去掉注释
        }
    }
}

//饱和函数
float satfunc(float data, float Max)//就是计算是否超过最大值//应该同比例缩放
{
    if(abs(data)>Max) return ( data > 0 ) ? Max : -Max;//abs（）就是定义在标准库cmath中的。
    else return data;
}

// float satfunc(float *data1,float *data2,float Max)
// {
//     float *tmp_vel_max;
//     float *tmp_vel_min;
//     tmp_vel_max=abs(*data1>*abs(data2)?(data1):(data2));
//     tmp_vel_min=abs(*data1>*abs(data2)?(data2):(data1));
//     *tmp_vel_min=(Max/tmp_vel_max)*(*tmp_vel_min);
//     *tmp_vel_max=(*tmp_vel_max>0)?Max:-Max;
//     if(abs(tmp_vel_max)>Max)  
// }



void collision_avoidance(float target_x,float target_y)
{
    //2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside )
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    //3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);

    //速度限幅
    for (int i = 0; i < 2; i++)
    {
        vel_track[i] = satfunc(vel_track[i],vel_track_max);
    }
    vel_collision[0]= 0;
    vel_collision[1]= 0;

    //4. 避障策略
    if(flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(angle_c/180*3.1415926);
        distance_cy = distance_c * sin(angle_c/180*3.1415926);

        float F_c;

        F_c = 0;

        if(distance_c > R_outside)
        {
            //对速度不做限制
            vel_collision[0] = vel_collision[0] + 0;
            vel_collision[1] = vel_collision[1] + 0;
            cout << " Forward Outside "<<endl;
        }

        //小幅度抑制移动速度
        if(distance_c > R_inside && distance_c <= R_outside)
        {
            F_c = p_R * (R_outside - distance_c);

        }

        //大幅度抑制移动速度
        if(distance_c <= R_inside )
        {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }//0.6  1.2

        if(distance_cx > 0)
        {
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }else{
            vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        }//这两玩意在上面设为0

        if(distance_cy > 0)
        {
            vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        }else{
            vel_collision[1] = vel_collision[1] - F_c * distance_cy /distance_c;
        }//这里也是设为0//if和else什么区别


        //避障速度限幅
        for (int i = 0; i < 2; i++)
        {
            vel_collision[i] = satfunc(vel_collision[i],vel_collision_max);//我自己在上面写了个函数。
        }
    }
    rotation_yaw(Euler_fcu[2],vel_collision,vel_collision);
    vel_sp_body[0] = vel_track[0] + vel_collision[0];
    vel_sp_body[1] = vel_track[1] + vel_collision[1]; //dyx

    //找当前位置到目标点的xy差值，如果出现其中一个差值小，另一个差值大，
    //且过了一会还是保持这个差值就开始从差值入手。
    //比如，y方向接近0，但x还差很多，但x方向有障碍，这个时候按discx cy的大小，缓解y的难题。

    for (int i = 0; i < 2; i++)
    {
        vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
    }
    rotation_yaw(Euler_fcu[2],vel_sp_body,vel_sp_ENU);//这是切换坐标系，但是应该放在速度相加之上。
    //这里的欧拉角2就是偏转角
}

void printf()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Minimun_distance : "<<endl;
    cout << "Distance : " << distance_c << " [m] "<<endl;
    cout << "Angle :    " << angle_c    << " [du] "<<endl;
    cout << "distance_cx :    " << distance_cx    << " [m] "<<endl;
    cout << "distance_cy :    " << distance_cy    << " [m] "<<endl;
    if(flag_collision_avoidance.data == true)
    {
        cout << "Collision avoidance Enabled "<<endl;
    }
    else
    {
        cout << "Collision avoidance Disabled "<<endl;
    }
    cout << "vel_track_x : " << vel_track[0] << " [m/s] "<<endl;
    cout << "vel_track_y : " << vel_track[1] << " [m/s] "<<endl;

    cout << "vel_collision_x : " << vel_collision[0] << " [m/s] "<<endl;
    cout << "vel_collision_y : " << vel_collision[1] << " [m/s] "<<endl;

    cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] "<<endl;
    cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] "<<endl;
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












