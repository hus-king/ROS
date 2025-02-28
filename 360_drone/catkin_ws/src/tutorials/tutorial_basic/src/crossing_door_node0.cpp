//box
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <actionlib_msgs/GoalID.h>
#include <XmlRpcValue.h>
#include "camera_processor/PointDepth.h"
#include <cmath>
#include <yaml.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0
#define TURNING_R 0
#define TURNING_L 1
#define TURNING_M 2

typedef struct AnswerBox{
    darknet_ros_msgs::BoundingBox bigbox;
    darknet_ros_msgs::BoundingBox smlbox[2];
    darknet_ros_msgs::BoundingBox cssbox[2];
    bool has_sml;
    bool flip_css;
}AnswerBox,*AnswerPtr;

AnswerBox TARGET;
darknet_ros_msgs::BoundingBox target_box[2];//目标棋盘格盒

int note[2];
int boxcount;
int no_target_time=0;
bool find_target=0;
bool missed_target;
int missed_time=0;
int rand_move_thresh;
float abs_distance;

//xy平面对齐阈值 例如 0.1(m)
float arrival_thresh=0.1;
//直接穿环z向阈值  例如1.0(m)
float just_go_through_thresh=1.0;
//斜向移动时z向阈值 例如1.5(m)
float slanting_move_thresh=1.5;
double dx,dy,dz;
//图片大小
int pic_size_x,pic_size_y;
//框大小差异阈值
float variable_thresh=0.5;//例如0.3---> x 0.7 ~ x 1.3
//框间距离差异阈值
float dist_thresh=1.0;//例如5倍框宽   注：实际方框为12倍框宽
float lean_angle;
float fx = 606.91064453125;
float fy = 606.9465942382812;
float cx = 317.84600830078125;
float cy = 237.4210662841797;
bool onetouch = true;

float comp_conf_thres=0.5;//大型框匹配置信度阈值
float norm_conf_thres=0.4;//方框匹配置信度阈值
float css_conf_thres=0.4;
float ovl_rate_low_thres=0.5;
float ovl_rate_high_thres=1.1;

float depth_thres = 0.3 ;
float far_thres = 2.5;

float gain_z(float x1, float y1, float x2, float y2) ;

float far_distance = 2.0f; //turning way时远距离点丢弃
std::vector<float> depth_info; //store depth info 
float depth_msg[640][480];
float global_vision_angle = 0;
float vision_distance = 0;

int turning = 0;
int CompWhichIsToBelieve(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int choice1,int choice2);
bool AutoMatchForSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag);
bool MatchForBothCurCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag);
bool MatchForBothCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag);
bool MatchForCompRing(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag);
bool MatchForBothSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int flag,int &note1,int &note2);
bool OneCornerIsIn(darknet_ros_msgs::BoundingBox bigbox,darknet_ros_msgs::BoundingBox smlbox);
void ABConstructor(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int bigbox,int smlbox1,int smlbox2,int chessbox1,int chessbox2);
bool MatchForOneSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &access);
float CalculateOverlapArea(float xmin1, float ymin1, float xmax1, float ymax1, float xmin2, float ymin2, float xmax2, float ymax2);
void TimeSurvel();
float CalculateOverlapRate(darknet_ros_msgs::BoundingBox bbox1,darknet_ros_msgs::BoundingBox bbox2);
void CalculateCoordinate();
bool JudgeIfPosAvail();
float gain_depth();


double getLengthBetweenPoints(geometry_msgs::Point a, double x, double y, double z,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    double err_x = a.x - x;
    double err_y = a.y - y;
    double err_z = a.z - z;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
}

double getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    return getLengthBetweenPoints(a, b.x, b.y, b.z, out_err_x, out_err_y, out_err_z);
}

//世界坐标系为左手系。向前为x正向，向左为y正向，向上为z正向
double getDistanceYaw(geometry_msgs::Point a,geometry_msgs::Point b)
{
    float dy,dx,angle;
    dy = b.y-a.y;
    dx = b.x-a.x;
    angle = atan2(dy,dx);//计算 a 到 b 的方向角，该角度是从x轴起，逆时针方向到 a 到 b 的直线的夹角。
    std::cout<<"dx: "<< dx <<"dy: "<< dy << "angle: "<<angle<<std::endl;
    return angle;
}


int Turning_Way(){

        float depth_sums[640];
        float depth_aver[640];

        int depth_index = 0;
        for (int x = 0; x < 640; ++x) {
            depth_index = 0;
            for (int y = 0; y < 480; ++y) {
                float depth_value = depth_msg[x][y];
                if (depth_value > 0.5 && depth_value <= far_distance) {
                    depth_sums[x] += depth_value;
                    depth_index ++;
                }
            }
            depth_aver[x] = depth_sums[x]/depth_index;
        }

        float left_sum = 0, right_sum = 0;

        for (int i = 1; i < 640; ++i) {
            if(depth_aver[i]>=depth_aver[i-1]) right_sum++;
            else if(depth_aver[i]<depth_aver[i-1]) left_sum++;
        }

        float rate = 0;
        rate = right_sum/(right_sum+left_sum);

        std::cout << "Left sum: " << left_sum << ", Right sum: " << right_sum << std::endl;
        if (rate > 0.52) {
            return TURNING_L;
        } else if(rate < 0.48){
            return TURNING_R;
        }
        else{
            return TURNING_M;
        }
}




mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(current_rpy.x, current_rpy.y, current_rpy.z);
}


//void camera_cb(const camera_processor::PointDepth::ConstPtr &msg) {
//    depth_info.clear();
//    depth_info.assign(msg->depths.begin(),msg->depths.end());
//}
void camera_cb(const camera_processor::PointDepth::ConstPtr &msg) {
    //depth_msg_ptr = &msg;
    for(int y=0;y<480;++y){
        for(int x=0;x<640;++x){
            depth_msg[x][y] = msg->depths[y*640+x];
        }
    }
//    ROS_INFO("success");
    //ROS_INFO("index depth:%f",depth_msg[320][180]);
    //printf("msg.size():%d",msg->depths.size());
}
/*bool check_if_Lb_Rr(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int flag1,int flag2){
        float xmin0= msg->bounding_boxes[flag1].xmin,xmax0=msg->bounding_boxes[flag1].xmax,xmin1=msg->bounding_boxes[flag2].xmin,xmax1=msg->bounding_boxes[flag2].xmax;
        float xcen0=(xmax0+xmin0)/2.0,xcen1=(xmax1+xmin1)/2.0,ycen0=(ymax0+ymin0)/2.0,ycen1=(ymax1+ymin1)/2.0;
    if(xcen0<xcen1){
    	if(msg->bounding_boxes[flag1].id==1&&msg->bounding_boxes[flag2].id==0){
    	return true;
    	}
    	if(msg->bounding_boxes[flag1].id==7&&msg->bounding_boxes[flag2].id==6){
    	return true;
    	}
    }
    else{
    	if(msg->bounding_boxes[flag1].id==0&&msg->bounding_boxes[flag2].id==1){
    	return true;
    	}
    	if(msg->bounding_boxes[flag1].id==6&&msg->bounding_boxes[flag2].id==7){
    	return true;
    	}
    }
    return false;
}
*/
void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
    find_target=false;
    int flag=-1;
    int both_square[2]={-1,-1};
    int both_chessboard[2]={-1,-1};
    if(MatchForCompRing(msg, flag)){
        switch(msg->bounding_boxes[flag].id){
            case 2:{
                if(MatchForBothCurCB(msg,both_chessboard[0],both_chessboard[1],flag)){
                    find_target=true;
                    missed_target=false;
                    ABConstructor(msg,flag,-1,-1,both_chessboard[0],both_chessboard[1]);
                    return;
                }
                else{
                    find_target=false;
                    missed_target=true;
                    return;
                }
                break;
            }
            //暂时放在一起，之后加方框位置判断后再分开吧
            case 4:case 5:{
                if(MatchForBothSquare(msg,flag,both_square[0],both_square[1])){
                    if(AutoMatchForSquare(msg,both_chessboard[0],both_chessboard[1],CompWhichIsToBelieve(msg,both_square[0],both_square[1]))){
                        if(msg->bounding_boxes[flag].id==5){
                            TARGET.flip_css=true;
                        }
                    }
                    else{
                        find_target=false;
                        missed_target=true;
                        return;
                    }
                }
                else if(MatchForOneSquare(msg,flag)){
                    if(MatchForBothCB(msg,both_chessboard[0],both_chessboard[1],flag)){
                        find_target=true;
                        missed_target=false;
                        ABConstructor(msg,flag,-1,-1,both_chessboard[0],both_chessboard[1]);
                        return;
                    }
                }
                else{
                    find_target=false;
                    missed_target=true;
                    return;
                }
    
                break;
            }
            default:{
                ROS_ERROR("False type when matching for complicated rings");
                break;
            }
        }

    }
    else if(MatchForOneSquare(msg,flag)){
        if(MatchForBothCB(msg,both_chessboard[0],both_chessboard[1],flag)){
            find_target=true;
            missed_target=false;
            ABConstructor(msg,flag,-1,-1,both_chessboard[0],both_chessboard[1]);
            return;
        }
        else{
            find_target=false;
            missed_target=true;
        }
    }
    else{
        ROS_WARN("Target Not Found!");
        find_target=false;
        missed_target=false;
    }
}

int CompWhichIsToBelieve(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int choice1,int choice2){
    boxcount=msg->bounding_boxes.size();
    if(choice1<0||choice2<0||choice1>=boxcount||choice2>=boxcount){
        return -1;
    }
    if(msg->bounding_boxes[choice1].probability>msg->bounding_boxes[choice2].probability){
        return choice1;
    }
    else{
        return choice2;
    }
}

bool AutoMatchForSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag){
    if(MatchForOneSquare(msg,flag)){
        if(MatchForBothCB(msg,flag1,flag2,flag)){
            find_target=true;
            missed_target=false;
            ABConstructor(msg,flag,-1,-1,flag1,flag2);
            return true;
        }
        else{
            find_target=false;
            missed_target=true;
            return false;
        }
    }
}


bool MatchForBothCurCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag){
    boxcount=msg->bounding_boxes.size();
    float promax1 = -1.0, promax2=-1.0;
    int ID;float rate;
    flag1=flag2=-1;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
        rate=CalculateOverlapRate(msg->bounding_boxes[flag],msg->bounding_boxes[i]);
	    if((ID==6)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax2=msg->bounding_boxes[i].probability;
            flag2=i;
	    }
        if((ID==7)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax1=msg->bounding_boxes[i].probability;
            flag1=i;
	    }
	}
    if(flag1>=0&&flag2>=0&&promax2>=css_conf_thres&&promax1>=css_conf_thres){
        return true;
    }
    else{
        return false;
    }
}
bool MatchForBothCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag1,int &flag2,int flag){
    boxcount=msg->bounding_boxes.size();
    float promax1 = -1.0, promax2=-1.0;
    int ID;float rate;
    flag1=flag2=-1;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
        rate=CalculateOverlapRate(msg->bounding_boxes[flag],msg->bounding_boxes[i]);
	    if((ID==0)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax2=msg->bounding_boxes[i].probability;
            flag2=i;
	    }
        if((ID==1)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax1=msg->bounding_boxes[i].probability;
            flag1=i;
	    }
	}
    if(flag1>=0&&flag2>=0&&promax2>=css_conf_thres&&promax1>=css_conf_thres){
        return true;
    }
    else{
        return false;
    }
}

bool MatchForCompRing(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &flag){
    boxcount=msg->bounding_boxes.size();
    float promax = -1.0;
    int ID;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
		if(msg->bounding_boxes[i].probability>promax&&(ID==2||ID==4||ID==5)){
            promax=msg->bounding_boxes[i].probability;
            flag=i;
		}
	}
    if(flag>=0&&promax>=comp_conf_thres){
        return true;
    }
    else{
        return false;
    }
}

bool MatchForBothSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int flag,int &note1,int &note2){
    boxcount=msg->bounding_boxes.size();
    int ID;
    float rate;
    note1=note2=-1;
    float promax = -1.0;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
        rate=CalculateOverlapRate(msg->bounding_boxes[flag],msg->bounding_boxes[i]);
		if((ID==3)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax=msg->bounding_boxes[i].probability;
            note1=i;
		}
	}
    promax=-1.0;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
        rate=CalculateOverlapRate(msg->bounding_boxes[flag],msg->bounding_boxes[i]);
		if(i!=note1&&(ID==3)&&rate>=ovl_rate_low_thres&&rate<=ovl_rate_high_thres){
            promax=msg->bounding_boxes[i].probability;
            note2=i;
		}
	}
    if(note1>=0&&note2>=0&&msg->bounding_boxes[note1].probability>=norm_conf_thres&&msg->bounding_boxes[note2].probability>=norm_conf_thres){
        return true;
    }
    else{
        return false;
    }
}

bool OneCornerIsIn(darknet_ros_msgs::BoundingBox bigbox,darknet_ros_msgs::BoundingBox smlbox){
    if(bigbox.xmin<=smlbox.xmin<=bigbox.xmax){
        return true;
    }
    if(bigbox.xmin<=smlbox.xmax<=bigbox.xmax){
        return true;
    }
    if(bigbox.ymin<=smlbox.ymax<=bigbox.ymax){
        return true;
    }
    if(bigbox.ymin<=smlbox.ymin<=bigbox.ymax){
        return true;
    }
    return false;
}
void ABConstructor(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int bigbox,int smlbox1,int smlbox2,int chessbox1,int chessbox2){
    if(msg->bounding_boxes[bigbox].id==2||msg->bounding_boxes[bigbox].id==3){
        TARGET.has_sml=false;
    }
    else{
        TARGET.has_sml=true;
        TARGET.smlbox[0]=msg->bounding_boxes[smlbox1];
        TARGET.smlbox[1]=msg->bounding_boxes[smlbox2];
    }
    TARGET.bigbox=msg->bounding_boxes[bigbox];
    TARGET.flip_css=false;
    TARGET.cssbox[0]=msg->bounding_boxes[chessbox1];
    TARGET.cssbox[1]=msg->bounding_boxes[chessbox2];
    
    return;
}
bool MatchForOneSquare(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg,int &access){
    boxcount=msg->bounding_boxes.size();
    int ID,flag=-1;
    float rate;
    float promax = -1.0;
    for(int i=0;i<boxcount;i++){
        ID=msg->bounding_boxes[i].id;
		if(msg->bounding_boxes[i].probability>promax&&ID==3){
            promax=msg->bounding_boxes[i].probability;
            flag=i;
		}
	}
    if(promax>=norm_conf_thres&&flag>=0){
        access=flag;
        return true;
    }
    else{
        return false;
    }
}
float CalculateOverlapArea(float xmin1, float ymin1, float xmax1, float ymax1, float xmin2, float ymin2, float xmax2, float ymax2){
    //计算重叠区域的左上角和右下角
    float overlap_xmin = std::max(xmin1, xmin2);
    float overlap_ymin = std::max(ymin1, ymin2);
    float overlap_xmax = std::min(xmax1, xmax2);
    float overlap_ymax = std::min(ymax1, ymax2);
    float overlap_area;
    if(overlap_xmin<overlap_xmax&&overlap_ymin<overlap_ymax){
        overlap_area = (overlap_xmax - overlap_xmin) * (overlap_ymax - overlap_ymin);
    }
    else{
        overlap_area=0.0;
    }
    //ROS_INFO("OverlapArea:%f",overlap_area);
    return overlap_area;
}
void TimeSurvel(){
    if(find_target){
        missed_target = false;
        missed_time = 0;
        no_target_time = 0;
    }
    else{
        no_target_time++;
    }
    if(missed_target){
        missed_time++;
    }
    return;
}
float CalculateOverlapRate(darknet_ros_msgs::BoundingBox bbox1,darknet_ros_msgs::BoundingBox bbox2){
    float overlap_area=CalculateOverlapArea(bbox1.xmin,bbox1.ymin,bbox1.xmax,bbox1.ymax,bbox2.xmin,bbox2.ymin,bbox2.xmax,bbox2.ymax);
    float area;

    float rate;
    float zone1=(bbox1.xmax-bbox1.xmin)*(bbox1.ymax-bbox1.ymin);
    float zone2=(bbox2.xmax-bbox2.xmin)*(bbox2.ymax-bbox2.ymin);
    if(zone2>zone1){
        area=(bbox1.xmax-bbox1.xmin)*(bbox1.ymax-bbox1.ymin);
    }
    else{
        area=(bbox2.xmax-bbox2.xmin)*(bbox2.ymax-bbox2.ymin);
    }
    if(area!=0){
        rate=overlap_area / area;
    }
    else{
        rate=-1.0;
    }
    //ROS_INFO("OverlapRate:%f",rate);
    return rate;
}  
//NOTE：更换计算坐标函数
void CalculateCoordinate(){
    float pic_center_x,pic_center_y;
    if(TARGET.flip_css){
        pic_center_x=(TARGET.bigbox.xmin+TARGET.bigbox.xmax)/2.0;
        pic_center_y=(TARGET.bigbox.ymin+TARGET.bigbox.ymax)/2.0;
    }
    else{
        float ymin0= TARGET.cssbox[0].ymin,ymax0= TARGET.cssbox[0].ymax,ymin1=TARGET.cssbox[1].ymin,ymax1=TARGET.cssbox[1].ymax,xmin0= TARGET.cssbox[0].xmin,xmax0= TARGET.cssbox[0].xmax,xmin1=TARGET.cssbox[1].xmin,xmax1=TARGET.cssbox[1].xmax;
        float xcen0=(xmax0+xmin0)/2.0,xcen1=(xmax1+xmin1)/2.0,ycen0=(ymax0+ymin0)/2.0,ycen1=(ymax1+ymin1)/2.0;
        pic_center_x=(xcen0+xcen1)/2.0;pic_center_y=(ycen0+ycen1)/2.0;
    }
	dz=gain_depth();
    dy=-dz*(pic_center_x-cx)/fx;
	dx=-dz*(pic_center_y-cy)/fy;
    abs_distance = sqrt(dx*dx+dy*dy);
    
  
    return;
}

float gain_depth(){
   
    float depth_sum = 0;
    float depth_average = 0;
    int e_index = 0;
    float x_middle = 0;
    float y_middle = 0;
    float x_smiddle = 0;
    float y_smiddle = 0;//four point
    float x_tmiddle = 0;//three point
    float z1,z2;
    if(TARGET.bigbox.id == 3){ //square
        if(TARGET.cssbox[0].id == 1){ //blue chess
            x_middle = (TARGET.cssbox[0].xmin + TARGET.cssbox[0].xmax)/2.0;
            y_middle = (TARGET.cssbox[0].ymin + TARGET.cssbox[0].ymax)/2.0;
            x_smiddle = (x_middle + TARGET.cssbox[0].xmin)/2.0;
            y_smiddle = (TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/8.0;
            
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/2.0;j++){
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[0].xmax - TARGET.cssbox[0].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }

            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_smiddle);i < static_cast<int>(x_middle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j] - depth_average) >= depth_thres ) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }

            z1 = depth_average;
            ROS_ERROR("Z1:%f",z1);
        }
        else if(TARGET.cssbox[0].id == 0){ //red chess
            x_middle = (TARGET.cssbox[0].xmin + TARGET.cssbox[0].xmax)/2.0;
            y_middle = (TARGET.cssbox[0].ymin + TARGET.cssbox[0].ymax)/2.0;
            x_smiddle = (x_middle + TARGET.cssbox[0].xmax)/2.0;
            y_smiddle = (TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/8.0;
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/2.0;j++){
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[0].xmax - TARGET.cssbox[0].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_middle);i < static_cast<int>(x_smiddle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j]-depth_average) >= depth_thres) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z2 = depth_average;
            ROS_ERROR("Z2:%f",z2);
        }
        
        float depth_sum = 0;
        float depth_average = 0;
        int e_index = 0;
        
        if(TARGET.cssbox[1].id == 1){ //blue chess
            x_middle = (TARGET.cssbox[1].xmin + TARGET.cssbox[1].xmax)/2.0;
            y_middle = (TARGET.cssbox[1].ymin + TARGET.cssbox[1].ymax)/2.0;
            x_smiddle = (x_middle + TARGET.cssbox[1].xmin)/2.0;
            y_smiddle = (TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/8.0;
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/2.0;j++){
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[1].xmax - TARGET.cssbox[1].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_smiddle);i < static_cast<int>(x_middle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j] - depth_average) >= depth_thres ) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }

            z1 = depth_average;
            ROS_ERROR("Z1:%f",z1);
        }
        else if(TARGET.cssbox[1].id == 0){ //red chess
            x_middle = (TARGET.cssbox[1].xmin + TARGET.cssbox[1].xmax)/2.0;
            y_middle = (TARGET.cssbox[1].ymin + TARGET.cssbox[1].ymax)/2.0;
            x_smiddle = (x_middle + TARGET.cssbox[1].xmax)/2.0;
            y_smiddle = (TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/8.0;
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/2.0;j++){
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[1].xmax - TARGET.cssbox[1].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_middle);i < static_cast<int>(x_smiddle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j]-depth_average) >= depth_thres) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z2 = depth_average;
            ROS_ERROR("Z2:%f",z2);
        }
    }
    else if(TARGET.bigbox.id == 2){ //circle
        if(TARGET.cssbox[0].id == 7){ //blue cur
            x_tmiddle = (-TARGET.cssbox[0].xmin + TARGET.cssbox[0].xmax) / 3.0 + TARGET.cssbox[0].xmin;
            x_middle = (TARGET.cssbox[0].xmax+TARGET.cssbox[0].xmin)/2.0;
            y_middle = (TARGET.cssbox[0].ymin + TARGET.cssbox[0].ymax) / 2.0;
            x_smiddle = (x_tmiddle + TARGET.cssbox[0].xmin) / 2.0 ; //  1/6
            y_smiddle = (TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin) / 8.0;//  1/8

            for(int j = 0;j < static_cast<int>(TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/2.0;j++) {
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[0].xmax - TARGET.cssbox[0].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }

            e_index ++;
            depth_average = depth_sum / e_index;
            for(int i = static_cast<int>(x_smiddle) ; i < static_cast<int>(x_tmiddle);i++){
                for(int j = static_cast<int>(y_middle- y_smiddle) ; j < static_cast<int>(y_middle + y_smiddle) ; j++){
                    if(fabs(depth_msg[i][j]-depth_average) >= depth_thres) continue;
                    depth_sum += depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z1 = depth_average;
        }
        else if(TARGET.cssbox[0].id == 6){ //red cur
            x_tmiddle = (-TARGET.cssbox[0].xmin + TARGET.cssbox[0].xmax) * 2.0 / 3.0 + TARGET.cssbox[0].xmin;
            x_middle = (TARGET.cssbox[0].xmax+TARGET.cssbox[0].xmin)/2.0;
            y_middle = (TARGET.cssbox[0].ymin + TARGET.cssbox[0].ymax) / 2.0;
            x_smiddle = (x_tmiddle + TARGET.cssbox[0].xmax) / 2.0; //  1/6
            y_smiddle = (TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin) / 8.0;  // 1/8
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[0].ymax - TARGET.cssbox[0].ymin)/2.0;j++) {
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[0].xmax - TARGET.cssbox[0].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_tmiddle);i < static_cast<int>(x_smiddle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j]-depth_average)>=depth_thres) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z2 = depth_average;
        }
        
        float depth_sum = 0;
        float depth_average = 0;
        int e_index = 0;

       if(TARGET.cssbox[1].id == 7){ //blue cur
            x_tmiddle = (-TARGET.cssbox[1].xmin + TARGET.cssbox[1].xmax) / 3.0 + TARGET.cssbox[1].xmin;
            x_middle = (TARGET.cssbox[1].xmax+TARGET.cssbox[1].xmin)/2.0;
            y_middle = (TARGET.cssbox[1].ymin + TARGET.cssbox[1].ymax) / 2.0;
            x_smiddle = (x_tmiddle + TARGET.cssbox[1].xmin) / 2.0 ; //  1/6
            y_smiddle = (TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin) / 8.0;//  1/8
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/2.0;j++) {
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[1].xmax - TARGET.cssbox[1].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum / e_index;
            for(int i = static_cast<int>(x_smiddle) ; i < static_cast<int>(x_tmiddle);i++){
                for(int j = static_cast<int>(y_middle- y_smiddle) ; j < static_cast<int>(y_middle + y_smiddle) ; j++){
                    if(fabs(depth_msg[i][j]-depth_average) >= depth_thres) continue;
                    depth_sum += depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z1 = depth_average;
        }
        else if(TARGET.cssbox[1].id == 6){ //red cur
            x_tmiddle = (-TARGET.cssbox[1].xmin + TARGET.cssbox[1].xmax) * 2.0 / 3.0 + TARGET.cssbox[1].xmin;
            x_middle = (TARGET.cssbox[1].xmax+TARGET.cssbox[1].xmin)/2.0;
            y_middle = (TARGET.cssbox[1].ymin + TARGET.cssbox[1].ymax) / 2.0;
            x_smiddle = (x_tmiddle + TARGET.cssbox[1].xmax) / 2.0; //  1/6
            y_smiddle = (TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin) / 8.0;  // 1/8
            for(int j = 0;j < static_cast<int>(TARGET.cssbox[1].ymax - TARGET.cssbox[1].ymin)/2.0;j++) {
                for(int i = 0 ;i < static_cast<int>( (TARGET.cssbox[1].xmax - TARGET.cssbox[1].xmin)/2.0 ) ; i++) {
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle + i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle + i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle+j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle+j)];
                        e_index ++;
                        }
                    if(depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]<=0.5 || depth_msg[static_cast<int>(x_tmiddle - i)][static_cast<int>(y_middle-j)]>=far_thres) {}
                    else{   
                        depth_sum += depth_msg[static_cast<int>(x_middle - i)][static_cast<int>(y_middle-j)];
                        e_index ++;
                        }
                    if(e_index!=0) break;
                }
                if(e_index!=0) break;
            }
            e_index ++;
            depth_average = depth_sum/e_index;
            for(int i = static_cast<int>(x_tmiddle);i < static_cast<int>(x_smiddle);i++){
                for(int j = static_cast<int>(y_middle - y_smiddle);j < static_cast<int>(y_middle + y_smiddle);j++){
                    if(fabs(depth_msg[i][j]-depth_average)>=depth_thres) continue;
                    depth_sum +=depth_msg[i][j];
                    e_index ++;
                    depth_average = depth_sum/e_index;
                }
            }
            z2 = depth_average;
        }
    }
    if(z1 - z2 > 0.5) turning = TURNING_R;
    else if(z1 - z2 < -0.5) turning = TURNING_L;
    else turning = TURNING_M;
    ROS_INFO("z1:%f,z2:%f",z1,z2);
    float xcen1 = (TARGET.cssbox[1].xmax + TARGET.cssbox[1].xmin)/2.0;
    float xcen0 = (TARGET.cssbox[0].xmax + TARGET.cssbox[0].xmin)/2.0;
    float css_dis = abs(xcen0 - xcen1)*(z1+z2)/(fx*2.0);
    lean_angle = atan2(z2-z1 , css_dis);
    return (z1+z2)/2 ;
}


bool JudgeIfPosAvail(){
    float ymin0= TARGET.cssbox[0].ymin,ymax0= TARGET.cssbox[0].ymax,ymin1=TARGET.cssbox[1].ymin,ymax1=TARGET.cssbox[1].ymax,xmin0= TARGET.cssbox[0].xmin,xmax0= TARGET.cssbox[0].xmax,xmin1=TARGET.cssbox[1].xmin,xmax1=TARGET.cssbox[1].xmax;
    float xcen0=(xmax0+xmin0)/2.0,xcen1=(xmax1+xmin1)/2.0,ycen0=(ymax0+ymin0)/2.0,ycen1=(ymax1+ymin1)/2.0;
    float size0=(ymax0-ymin0)*(xmax0-xmin0),size1=(ymax1-ymin1)*(xmax1-xmin1);
    if(size1==0||(abs(size0/size1)>(1+variable_thresh))||(abs(size0/size1)<(1-variable_thresh))){
        return false;
    }
    if((abs(xcen1-xcen0)/abs(xmax1-xmin1+xmax0-xmin0)/2.0)<dist_thresh){
        return false;
    }
    return true;
}


bool judge_if_position_correct() {
    float ymin0 = target_box[0].ymin, ymax0 = target_box[0].ymax, ymin1 = target_box[1].ymin, ymax1 = target_box[1].ymax;
    float xmin0 = target_box[0].xmin, xmax0 = target_box[0].xmax, xmin1 = target_box[1].xmin, xmax1 = target_box[1].xmax;
    float xcen0 = (xmax0 + xmin0) / 2.0, xcen1 = (xmax1 + xmin1) / 2.0;
    float size0 = (ymax0 - ymin0) * (xmax0 - xmin0), size1 = (ymax1 - ymin1) * (xmax1 - xmin1);

    if ((size0 / size1 > (1 + variable_thresh)) || (size0 / size1 < (1 - variable_thresh))) {
        return false;
    }

    if (((xcen1 - xcen0) / ((xmax1 - xmin1 + xmax0 - xmin0) / 2.0)) < dist_thresh) {
        return false;
    }

    ROS_INFO("Position is correct!\n");
    return true;
}


geometry_msgs::Twist move_base_twist;
void move_base_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
    move_base_twist = *msg;
}

geometry_msgs::Point last_err;
geometry_msgs::Point err_sum;
double last_yaw_err = 0.;
double yaw_err_sum = 0.;
ros::Time last_pid_control_time;
geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target,double reach_yaw) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        err_sum.x = 0.;
        err_sum.y = 0.;
        err_sum.z = 0.;
        yaw_err_sum = 0.;
    }

    geometry_msgs::Point err;
    double absErr = getLengthBetweenPoints(target, current_pose.pose.position, &err.x, &err.y, &err.z);

    double y_err = reach_yaw - current_rpy.z;
    double dy_err = (y_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = VEL_P * y_err + VEL_I * yaw_err_sum + VEL_D * dy_err;

    if (absErr > 0.8) {
        ret.linear.x = err.x * 0.8 / absErr;
        ret.linear.y = err.y * 0.8 / absErr;
        ret.linear.z = err.z * 0.8 / absErr;

        err_sum.x = .0;
        err_sum.y = .0;
        err_sum.z = .0;
    } else {
        geometry_msgs::Point d_err;
        d_err.x = (err.x - last_err.x) / dt.toSec();
        d_err.y = (err.y - last_err.y) / dt.toSec();
        d_err.z = (err.z - last_err.z) / dt.toSec();

        ret.linear.x = VEL_P * err.x + VEL_I * err_sum.x + VEL_D * d_err.x;
        ret.linear.y = VEL_P * err.y + VEL_I * err_sum.y + VEL_D * d_err.y;
        ret.linear.z = VEL_P * err.z + VEL_I * err_sum.z + VEL_D * d_err.z;

        err_sum.x += err.x * dt.toSec();
        err_sum.y += err.y * dt.toSec();
        err_sum.z += err.z * dt.toSec();
    }

    last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    return ret;
}

struct HoverTarget {
    std::vector<geometry_msgs::Point> points;
    std::vector<float> yaw;
};

float fly_height;
float vision_range = 2.0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "crossing_door_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber camera_sub = nh.subscribe<camera_processor::PointDepth>("/camera_processor/depth/points", 1, camera_cb);
    ros::Subscriber move_base_cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, move_base_cmd_vel_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb);
    no_target_time = 0;
    find_target = false;
    // Get hovering location in parameters from YAML
    HoverTarget hover_target;
    XmlRpc::XmlRpcValue hover_points_list;

    if (nh.getParam("hover_target", hover_points_list)) {
        ROS_ASSERT(hover_points_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int i = 0; i < hover_points_list.size(); ++i) {
            ROS_ASSERT(hover_points_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            ROS_ASSERT(hover_points_list[i].hasMember("x") && hover_points_list[i].hasMember("y") && hover_points_list[i].hasMember("z")&& hover_points_list[i].hasMember("yaw"));

            geometry_msgs::Point point;
            geometry_msgs::Point set_point;
            float cal_yaw;
            point.x = static_cast<double>(hover_points_list[i]["x"]);
            point.y = static_cast<double>(hover_points_list[i]["y"]);
            point.z = static_cast<double>(hover_points_list[i]["z"]);
            cal_yaw = static_cast<double>(hover_points_list[i]["yaw"]);
            
            set_point.x = point.x - vision_range*cos(cal_yaw);
            set_point.y = point.y - vision_range*sin(cal_yaw);
            set_point.z = point.z ;

            hover_target.points.push_back(set_point);
            hover_target.yaw.push_back(cal_yaw);
        }
    } else {
        ROS_WARN("Parameters not found, using default values.");

        // 使用默认值
        geometry_msgs::Point default_point;
        default_point.x = 2.0;
        default_point.y = 2.0;
        default_point.z = 0.0;
        hover_target.points.push_back(default_point);
    }
    
    nh.param<float>("fly_height",fly_height, 0.8);

    for(int i=0;i < hover_target.points.size();++i) {
        std::cout << "hover_x[" << i << "]:" << hover_target.points[i].x << " hover_y[" << i << "]:" << hover_target.points[i].y << " hover_z[" << i << "]:" << hover_target.points[i].z << " hover_z[" << i << "]:" << hover_target.yaw[i]<< std::endl;
        std::cout<<"fly_height:"<<fly_height<<std::endl;
    }
    nh.param<int>("no_target_time", no_target_time, 0);
    nh.param<bool>("find_target", find_target, false);
    nh.param<bool>("missed_target", missed_target, false);
    nh.param<int>("missed_time", missed_time, 0);
    nh.param<int>("rand_move_thresh", rand_move_thresh, 10);
    nh.param<float>("abs_distance", abs_distance, 5.0f);

    nh.param<float>("arrival_thresh", arrival_thresh, 0.1f);
    nh.param<float>("just_go_through_thresh", just_go_through_thresh, 1.0f);
    nh.param<float>("slanting_move_thresh", slanting_move_thresh, 1.5f);

    nh.param<int>("pic_size_x", pic_size_x, 640);
    nh.param<int>("pic_size_y", pic_size_y, 480);

    nh.param<float>("variable_thresh", variable_thresh, 0.5f);
    nh.param<float>("dist_thresh", dist_thresh, 1.0f);

    nh.param<float>("comp_conf_thres", comp_conf_thres, 0.5f);
    nh.param<float>("norm_conf_thres", norm_conf_thres, 0.4f);
    nh.param<float>("css_conf_thres", css_conf_thres, 0.4f);
    nh.param<float>("ovl_rate_low_thres", ovl_rate_low_thres, 0.5f);
    nh.param<float>("ovl_rate_high_thres", ovl_rate_high_thres, 1.1f);

    nh.param<float>("depth_thres", depth_thres, 0.3f);
    nh.param<float>("far_thres", far_thres, 2.5f);

    nh.param<float>("far_distance", far_distance, 2.0f);

    std::cout<<"no_target_time:"<<no_target_time<<std::endl;
    std::cout<<"missed_target:"<< missed_target<<std::endl;
    std::cout<<"missed_time:"<< missed_time<<std::endl;
    std::cout<<"rand_move_thresh:"<< rand_move_thresh<<std::endl;
    std::cout<<"abs_distance:"<< abs_distance<<std::endl;
    std::cout<<"arrival_thresh:"<< arrival_thresh<<std::endl;
    std::cout<<"slanting_move_thresh:"<< slanting_move_thresh<<std::endl;
    std::cout<<"just_go_through_thresh:"<< just_go_through_thresh<<std::endl;
    std::cout<<"pic_size_x:"<< pic_size_x<<std::endl;
    std::cout<<"pic_size_y:"<< pic_size_y<<std::endl;
    std::cout<<"variable_thresh:"<< variable_thresh<<std::endl;
    std::cout<<"dist_thresh:"<< dist_thresh<<std::endl;
    std::cout<<"comp_conf_thres:"<< comp_conf_thres<<std::endl;
    std::cout<<"norm_conf_thres:"<< norm_conf_thres<<std::endl;
    std::cout<<"css_conf_thres:"<< css_conf_thres<<std::endl;
    std::cout<<"ovl_rate_low_thres:"<< ovl_rate_low_thres<<std::endl;
    std::cout<<"ovl_rate_high_thres:"<< ovl_rate_high_thres<<std::endl;
    std::cout<<"depth_thres:"<< depth_thres<<std::endl;
    std::cout<<"far_thres:"<< far_thres<<std::endl;
    std::cout<<"far_distance:"<< far_distance<<std::endl;
    
   

    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    int fsm_state = 0;
    int finding_state =-1;
    ros::Time last_srv_request = ros::Time::now();
    int target_index = 0;
    double target_yaw = 0;
    geometry_msgs::Point set_now_position;
    geometry_msgs::Point vision_target;
    double set_now_yaw;
    float real_depth;
    target_yaw = getDistanceYaw(current_pose.pose.position,hover_target.points[target_index]);
    float record_time = 0;
    float square_view = 0;
    float find_time = 0;

    while (ros::ok()) {
        geometry_msgs::TwistStamped twist;
        //target_yaw = getDistanceYaw(hover_target.points[target_index],current_pose.pose.position);
        switch (fsm_state) {
            case 0:  // Before offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1;  // goto before armed state
                    last_srv_request = ros::Time::now();
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("Offboard enabled");
                        } else {
                            ROS_WARN("Failed to enable offboard");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;

            case 1:  // After offboard, before armed state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
                    
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = fly_height+0.1;
                    set_now_yaw = 0;
                    
                    last_srv_request = ros::Time::now();
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                        } else {
                            ROS_WARN("Failed to arm vehicle");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;

            case 2:  // Takeoff state
                if (current_pose.pose.position.z > fly_height) {
                    fsm_state = 3;  // goto navigation state
                    //geometry_msgs::PoseStamped move_base_msg;
                    //move_base_msg.header.frame_id = "map";
                    //move_base_msg.pose.position = hover_target.points[target_index];
                    //move_base_msg.pose.orientation.w = -1.0;
                    //goal_pub.publish(move_base_msg);
                } else {
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw); //pid take-off
                }
                break;

            case 3:  // Navigation state
               if (target_index >= hover_target.points.size()) {
                    fsm_state = 7;  // All points visited, goto hover state
                    last_srv_request = ros::Time::now();
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    continue;
                    //actionlib_msgs::GoalID cancel_msg;
                    //cancel_pub.publish(cancel_msg);
                       
                }
                if (getLengthBetweenPoints(hover_target.points[target_index], current_pose.pose.position) < 0.3) //到达预设点位
                {
                    //geometry_msgs::PoseStamped move_base_msg;
                    //move_base_msg.header.frame_id = "map";
                    //move_base_msg.pose.position = hover_target.points[target_index];
                    //move_base_msg.pose.orientation.w = -1.0;
                    //goal_pub.publish(move_base_msg);
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                    std::cout<<"arrived set position ,turning to face the box<<<<<<<<<<<<<<<<<<"<<std::endl;
                    fsm_state = 31;//finding target place TODO case31 turn to the box
                    finding_state = 2;

                } else {
                    //twist.twist = move_base_twist;
                    //twist.twist.linear.z = std::max(-0.5, std::min(0.5, hover_target.points[target_index].z - current_pose.pose.position.z));
                    //twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                    twist.twist = get_pid_vel(hover_target.points[target_index],target_yaw);
                }
                /*if(find_target){
                   set_now_position.x = current_pose.pose.position.x;
                   set_now_position.y = current_pose.pose.position.y;
                   set_now_position.z = current_pose.pose.position.z;
                   set_now_yaw = current_rpy.z;
                   twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                   fsm_state = 5;
                   finding_state = 1;//finding while navigation
                }*/
                
                break;
            case 31: //turning_case 转向直面目标框
                if (fabs(hover_target.yaw[target_index]-current_rpy.z) < 0.1) {

                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                    std::cout<<"finished turning , start finding the box"<<std::endl;
                    fsm_state = 5;//finding target place TODO case31 turn to the box
                    finding_state = 2;

                } else {
                    //twist.twist = move_base_twist;
                    //twist.twist.linear.z = std::max(-0.5, std::min(0.5, hover_target.points[target_index].z - current_pose.pose.position.z));
                    //twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                    twist.twist = get_pid_vel(set_now_position,hover_target.yaw[target_index]);
                }
                break;
            case 5: //finding_state 寻找目标框
                 //TODO Init params
                 TimeSurvel();
                 if(no_target_time > rand_move_thresh){
                     if(finding_state == 1){
                        finding_state = -1;
                        fsm_state = 3;
                     }else if(finding_state == 2) {
                        fsm_state = 63;//后退
                        vision_target.x = current_pose.pose.position.x - 0.5*cos(current_rpy.z);//给后退设置目标 此处target是预设的后退要到的点
                        vision_target.y = current_pose.pose.position.y - 0.5*sin(current_rpy.z);//YAML 未找到框但是到达预设点位时后退距离
                        vision_target.z = current_pose.pose.position.z ;
                        
                        finding_state = 3;
                        missed_target=false;
        				missed_time=0;
       					no_target_time=0;
       					
                        last_srv_request = ros::Time::now();
                        set_now_yaw = current_rpy.z;
                        std::cout<<"can't find"<<std::endl;
                        //TODO turning_case
                     }
                     else if(finding_state == 3) {
                     	fsm_state = 64;
                     	if(square_view == 0) {
                        	vision_target.x = current_pose.pose.position.x ;
                        	vision_target.y = current_pose.pose.position.y + 0.5;
                        	vision_target.z = current_pose.pose.position.z - 0.5;//方形移动路线左下角点
                        }
						missed_target=false;
        				missed_time=0;
       					no_target_time=0;
       					
                        last_srv_request = ros::Time::now();
                        set_now_yaw = current_rpy.z;
                        std::cout<<"can't find"<<std::endl;
					 }
					 else if(finding_state == 4){ //safe navigation
                     	fsm_state = 9; //避障飞到下一个点位
                        geometry_msgs::PoseStamped move_base_msg;
                        move_base_msg.header.frame_id = "map";
                        move_base_msg.pose.position = hover_target.points[target_index];
                        move_base_msg.pose.orientation.w = -1.0;
                        goal_pub.publish(move_base_msg);
                        
                        target_index ++;

                        square_view = 0 ;//切换目标点时将当前目标点的方形移动找点标识清零
                        finding_state = -1;
                        missed_target=false;
        				missed_time=0;
       					no_target_time=0;
       					
                        last_srv_request = ros::Time::now();
                        set_now_yaw = current_rpy.z;
                        std::cout<<"can't find"<<std::endl;
					 }
                 }
                 twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                 
                 if(find_target){
                      std::cout<<"find target --- ready to crossing" << std::endl;
                      twist.twist = get_pid_vel(set_now_position,set_now_yaw); 
                      CalculateCoordinate();
                      //if(JudgeIfPosAvail()){
                      ROS_ERROR("lean_angle:%f",lean_angle);
                        if(fabs(lean_angle) >= 0.26)  //YAML 面向框目标角15°
                        {
                            std::cout<<"adjust angle<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                            if (lean_angle < 0) {
                                 global_vision_angle = current_rpy.z +1.57 - fabs(lean_angle);
                                 vision_distance = fabs(-dy+dz*tan(3.14-fabs(lean_angle)))/sqrt(1+tan(3.14-fabs(lean_angle))*tan(3.14-fabs(lean_angle)));
                            }
                            else if (lean_angle > 0){
                                 global_vision_angle = current_rpy.z - (1.57-lean_angle);
                                 vision_distance = fabs(-dy+dz*tan(fabs(lean_angle)))/sqrt(1+tan(fabs(lean_angle))*tan(fabs(lean_angle))); 
                            }
                            vision_target.x = current_pose.pose.position.x + vision_distance*cos(global_vision_angle);
                            vision_target.y = current_pose.pose.position.y + vision_distance*sin(global_vision_angle);
                            vision_target.z = current_pose.pose.position.z + dx - 0.1;
                            real_depth = sqrt(dy*dy+dz*dz-vision_distance*vision_distance);
                            std::cout<<"vision_distance" << vision_distance << std::endl;
                            std::cout<<"vision_distance*cos(global_vision_angle)" << vision_distance*cos(global_vision_angle) << std::endl;
                            std::cout<<"vision_distance*sin(global_vision_angle)" << vision_distance*sin(global_vision_angle) << std::endl;
                            std::cout<<"vision_target.x" << vision_target.x << std::endl;
                            std::cout<<"vision_target.y" << vision_target.y << std::endl;
                            std::cout<<"vision_target.z" << vision_target.z << std::endl;
                            std::cout<<"real_depth" << real_depth << std::endl;

                            set_now_yaw = current_rpy.z + lean_angle;
                            std::cout<<"set_now_yaw" << current_rpy.z + lean_angle << std::endl;
                            fsm_state = 61;

                            // fsm_state = 7 ;

                            last_srv_request = ros::Time::now();
                            // twist.twist = get_pid_vel(vision_target,set_now_yaw); 
                        }
                        else{
                            std::cout<<"ready to crossing<<<<<<<<<<<<<<<<<<<<" << std::endl;

                            ROS_ERROR("dx:%f,dy:%f,dz:%f",dx,dy,dz);

                            vision_target.x = current_pose.pose.position.x + (dz+0.8)*cos(current_rpy.z) - dy*sin(current_rpy.z);
                            vision_target.y = current_pose.pose.position.y + (dz+0.8)*sin(current_rpy.z) + dy*cos(current_rpy.z);
                            vision_target.z = current_pose.pose.position.z + dx ;

                            ROS_ERROR("current:%f,currenty:%f,currentz:%f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
                            ROS_ERROR("visionx:%f,visiony:%f,visionz:%f",vision_target.x,vision_target.y,vision_target.z);

                            //set_now_yaw = current_rpy.z; //TODO turning
                            fsm_state = 62;

                            //fsm_state = 7 ;

                            last_srv_request = ros::Time::now();
                            // twist.twist = get_pid_vel(vision_target,set_now_yaw); 
                        }
                     // }
                    
                 }
                //  else{
                //     ROS_ERROR("-------------------not find target  ----------------------");
                //     ROS_ERROR("-----------------pid publish now_position-----------------");
                //    set_now_position.x = current_pose.pose.position.x;
                //    set_now_position.y = current_pose.pose.position.y;
                //    set_now_position.z = current_pose.pose.position.z;
                //    set_now_yaw = current_rpy.z;
                //    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                //  }
                break;
            case 61: //turning to box 转向直面方框
                if (getLengthBetweenPoints(vision_target, current_pose.pose.position) >= 0.1 || fabs(current_rpy.z - set_now_yaw)>=0.1 ) {
                   twist.twist = get_pid_vel(vision_target,set_now_yaw);
                }
                else {
                    std::cout<<"finish box"<<std::endl;
                    target_index++;
                    fsm_state = 62; //TODO set 3
                    last_srv_request = ros::Time::now();
                    //update hover_position
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    vision_target.x = current_pose.pose.position.x + (real_depth+1)*cos(set_now_yaw);
                    vision_target.y = current_pose.pose.position.y + (real_depth+1)*sin(set_now_yaw);
                    vision_target.z = current_pose.pose.position.z ;
                    std::cout<<"vision_target.x:"<<vision_target.x<<"vision_target.y:"<<vision_target.y<<"vision_target.z:"<<vision_target.z<<std::endl;
                    // if(target_index < hover_target.points.size()){ //TODO check "<" or "<="
                    //     target_yaw = getDistanceYaw(current_pose.pose.position,hover_target.points[target_index]);
                    // }
                }
                break;    

            case 62: //crossing_state 穿越框
                if (getLengthBetweenPoints(vision_target, current_pose.pose.position) >= 0.1) {
                   twist.twist = get_pid_vel(vision_target,set_now_yaw);
                } else {
                    std::cout<<"finish box"<<std::endl;
                    target_index++;
                    fsm_state = 7; //TODO set 3
                    last_srv_request = ros::Time::now();
                    //update hover_position
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    if(target_index < hover_target.points.size()){ //TODO check "<" or "<="
                        target_yaw = getDistanceYaw(current_pose.pose.position,hover_target.points[target_index]);
                    }
                }
                break; 

            case 63: //back to view box 后退观察方框
                if (getLengthBetweenPoints(vision_target, current_pose.pose.position) >= 0.1) {
                   twist.twist = get_pid_vel(vision_target,set_now_yaw);
                } 
                else {
                    std::cout<<"finished moving back , arrived back target point"<<std::endl;
                    ROS_ERROR("-----------------------------------------");
                    fsm_state = 7;  //TODO set 5
                    missed_target = false;
                    missed_time = 0;
                    no_target_time = 0;
                    last_srv_request = ros::Time::now();
                    finding_state = 3 ;
                    //update hover_position
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                   
                }
                break;  
                      
            case 64: //up or down to view box 上下移动识别方框
                  
                if (getLengthBetweenPoints(vision_target, current_pose.pose.position) >= 0.2) {
                   twist.twist = get_pid_vel(vision_target,set_now_yaw);
                   last_srv_request = ros::Time::now();
                } 
				if(ros::Time::now() - last_srv_request >= ros::Duration(5)) //在一个点位悬停超过5s
                {
                	if(record_time == 0) //移动到四边形下一个位置.
                	{
                		vision_target.x = current_pose.pose.position.x ;
                        vision_target.y = current_pose.pose.position.y ;
                        vision_target.z = current_pose.pose.position.z +1;
                        set_now_yaw = current_rpy.z;
                        twist.twist = get_pid_vel(vision_target,set_now_yaw);
                        record_time++;
					}
					else if(record_time == 1)
                	{
                		vision_target.x = current_pose.pose.position.x ;
                        vision_target.y = current_pose.pose.position.y -1;
                        vision_target.z = current_pose.pose.position.z ;
                        set_now_yaw = current_rpy.z;
                        twist.twist = get_pid_vel(vision_target,set_now_yaw);
                        record_time++;
					}
					else if(record_time == 2)
                	{
                		vision_target.x = current_pose.pose.position.x ;
                        vision_target.y = current_pose.pose.position.y ;
                        vision_target.z = current_pose.pose.position.z -1;
                        set_now_yaw = current_rpy.z;
                        twist.twist = get_pid_vel(vision_target,set_now_yaw);
                        record_time++;
					}
					else if(record_time == 3)
                	{
                		vision_target.x = current_pose.pose.position.x ;
                        vision_target.y = current_pose.pose.position.y +0.5;
                        vision_target.z = current_pose.pose.position.z +0.5;
                        set_now_yaw = current_rpy.z;
                        twist.twist = get_pid_vel(vision_target,set_now_yaw);
                        record_time++;
					}
					else if(record_time == 4){					
	                    std::cout<<"no target "<<std::endl;
	                    //update hover_position
	                    set_now_position.x = current_pose.pose.position.x;
	                    set_now_position.y = current_pose.pose.position.y;
	                    set_now_position.z = current_pose.pose.position.z;
	                    set_now_yaw = current_rpy.z;
	                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
	                    record_time++;
                    }
                    else if(record_time == 5){					
	                    fsm_state = 5; 
	                    last_srv_request = ros::Time::now();
	                    //update hover_position
	                    set_now_position.x = current_pose.pose.position.x;
	                    set_now_position.y = current_pose.pose.position.y;
	                    set_now_position.z = current_pose.pose.position.z;
	                    set_now_yaw = current_rpy.z;
	                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
	                    record_time = 0;
	                    square_view = 0;
	                    finding_state = 4;
                    }
                }
                else {
                    twist.twist = get_pid_vel(vision_target,set_now_yaw);
                }

				if(find_target) {
					set_now_position.x = current_pose.pose.position.x;
	                set_now_position.y = current_pose.pose.position.y;
	                set_now_position.z = current_pose.pose.position.z;
	                set_now_yaw = current_rpy.z;
	                twist.twist = get_pid_vel(set_now_position,set_now_yaw);//悬停

                    square_view = 1;
                    find_time++ ;
                    fsm_state = 5 ;
				}
				if(find_time >= 10 )
				{
				    fsm_state = 5; 
	                last_srv_request = ros::Time::now();
	                    //update hover_position
	                set_now_position.x = current_pose.pose.position.x;
	                set_now_position.y = current_pose.pose.position.y;
	                set_now_position.z = current_pose.pose.position.z;
	                set_now_yaw = current_rpy.z;
	                twist.twist = get_pid_vel(set_now_position,set_now_yaw);
	                record_time = 0;
	                square_view = 0;
	                finding_state = 4;	
	                find_time = 0;
				}
				
                break;    
                
            case 7:  // Hover state
                if (ros::Time::now() - last_srv_request > ros::Duration(5.0)) {
                    fsm_state = 8;  // goto land state
                    set_now_position.z = 0.1 ; 
                } else {  // PID control
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw); 
                }
                break;

            case 8:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.2) {
                    if (ros::Time::now() - last_srv_request > ros::Duration(0.5)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                        last_srv_request = ros::Time::now();
                    }
                } else {
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw); 
                } 
                break;
                
            case 9:  // 避障前往下一个点，放弃当前点位
            	if (getLengthBetweenPoints(hover_target.points[target_index], current_pose.pose.position) >= 0.1) {
                   twist.twist = move_base_twist;
                   twist.twist.linear.z = std::max(-0.5, std::min(0.5, hover_target.points[target_index].z - current_pose.pose.position.z));
                   twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                } else {
                    std::cout<<"arrived next target , state 31 : start turning"<<std::endl;
                    fsm_state = 31; 
                    last_srv_request = ros::Time::now();
                    //update hover_position
                    set_now_position.x = current_pose.pose.position.x;
                    set_now_position.y = current_pose.pose.position.y;
                    set_now_position.z = current_pose.pose.position.z;
                    set_now_yaw = current_rpy.z;
                    twist.twist = get_pid_vel(set_now_position,set_now_yaw);
                   
                }
                break;    
            

            default:
                fsm_state = -1;
                break;
        }

        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
