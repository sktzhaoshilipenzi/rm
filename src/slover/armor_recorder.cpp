/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "slover/armor_recorder.hpp"
#include "slover/angle_slover.hpp"
#include "utility/debug_utility.hpp"
namespace autocar
{
namespace slover
{
//#define use_predict
#define show_src_rect

const int bullet_speed = 18;     
const cv::Point ptoffset = cv::Point(0,0); // 子弹的偏移量 offset x → y ↓  
bool comp(const float &a, const float &b)
  {
      return abs(a) > abs(b);
  }
/**
 * @brief 定义一个距离函数，计算装甲片之间姿态的“距离 ”
 */
inline bool pos_distance(const vision_mul::armor_pos& pos1, const vision_mul::armor_pos& last_pos )
{
    return std::sqrt((pos1.angle_x - last_pos.angle_x) * (pos1.angle_x - last_pos.angle_x) + 
                     (pos1.angle_y - last_pos.angle_y) * (pos1.angle_y - last_pos.angle_y) +
                      std::abs(pos1.angle_z - last_pos.angle_z))/10;   
}
float Armor_recorder::PreFilter(float Pre_vel,float x_filter)
  {
    // Pre_vels_[filter_cnt%3] = Pre_vel;
    list_vx[0] = list_vx[1];
    list_vx[1] = list_vx[2];
    list_vx[2] = list_vx[3];
    list_vx[3] = list_vx[4];
    list_vx[4] = list_vx[5];
    list_vx[5] = list_vx[6];
    list_vx[6] = list_vx[7];
    list_vx[7] = list_vx[8];
    list_vx[8] = Pre_vel;

    list_vx_copy = list_vx;
    sort(list_vx_copy.begin(), list_vx_copy.end(), comp);
    // ROS_ERROR("%d", filter_cnt%9);
    list_x[0] = list_x[1];
    list_x[1] = list_x[2];
    list_x[2] = list_x[3];
    list_x[3] = list_x[4];
    list_x[4] = list_x[5];
    list_x[5] = list_x[6];
    list_x[6] = list_x[7];
    list_x[7] = list_x[8];
    list_x[8] = x_filter;

    list_x_copy = list_x;
    sort(list_x_copy.begin(), list_x_copy.end(), comp);
    return list_vx_copy[4];
    // else
    // return -Pre_vels_copy[4];
    
    
  }
float Armor_recorder::preFilter_for_z(float Pre_z)
{list_z[0] = list_z[1];
    list_z[1] = list_z[2];
    list_z[2] = list_z[3];
    list_z[3] = list_z[4];
    list_z[4] = list_z[5];
    list_z[5] = list_z[6];
    list_z[6] = Pre_z;

    list_z_copy = list_z;

    
    
    

}
float Armor_recorder::get_midum_z()
{ sort(list_z_copy.begin(), list_z_copy.end(), comp);
    // ROS_ERROR("%d", filter_cnt%9);
   // printf("\n%f==取值===\n",list_z_copy[3]);
    return list_z_copy[3];
 
}
void Armor_recorder::kalman_predict(cv::Point2f &last_point,cv::KalmanFilter &klaman,vision_mul::armor_info &armor_bf_predict,vision_mul::armor_info &armor_predict,bool &flag,float realxdiff,float realydiff,cv::RotatedRect &real_rect)
{   cv::Point2f relative_point=armor_bf_predict.rect.center;
    real_rect.size=armor_bf_predict.rect.size;
    real_rect.center=cv::Point2f(relative_point.x+realxdiff,relative_point.y+realydiff);
    real_rect.center=cv::Point2f(relative_point.x,relative_point.y);
    real_rect.angle=armor_bf_predict.rect.angle;
    cv::Point2f real_point=real_rect.center;
    cv::Mat processNoise(4, 1, CV_32F);   
    flag=true;
    if(last_point.x==0)
    {flag=false;
     max_vx=0;
     op_max_vx=0;
     his_max_vx=0;}
     printf("\n=========比例大小为%f\n",abs(last_point.x-real_point.x)/real_rect.size.width);
    if(abs(last_point.x-real_point.x)>=0.8*real_rect.size.width||
       abs(last_point.y-real_point.y)>=0.3*real_rect.size.height)
    {flag =false;
     //k=0;
     max_vx=0;
     op_max_vx=0;
     his_max_vx=0;
   //  printf("大幅跳变");
   }
  //  if (last_point.x!=0)
  //   {his_last_point=last_point;}
   // if(last_point.x==0&&his_last_point.x!=0)
   // last_point=his_last_point;
    if(k>=6)
    {
        k=0;
        max_vx=0;
        op_max_vx=0;
        his_max_vx=0;
    }
   // int predict_frame_dalay=5;  //反应延迟
    
    cv::Mat measurement= cv::Mat::zeros(2, 1, CV_32F);
    measurement=(cv::Mat_<float>(2,1)<<real_point.x,real_point.y);
    klaman.transitionMatrix = (cv::Mat_<float>(4,4) << 
                                           1,0,1,0,
                                           0,1,0,1,
                                           0,0,1,0,
                                           0,0,0,1
                                           );
setIdentity(klaman.measurementMatrix, cv::Scalar::all(1));
setIdentity(klaman.processNoiseCov, cv::Scalar::all(1e-1));
setIdentity(klaman.measurementNoiseCov, cv::Scalar::all(1e-4));
//if(k!=0)
//klaman.correct(measurement);
float ydiff=0;
float xdiff=0;
int scale=armor_bf_predict.state;
if(last_point.x!=0){
 xdiff=(real_point.x-last_point.x);
 ydiff=(real_point.y-last_point.y);}
 //printf("xdiff---------%f---------\n",xdiff);
int temp_vx=PreFilter(xdiff,real_point.x);
float x_mid=this->get_medum_x();
if(temp_vx>max_vx)
max_vx=temp_vx;
if(temp_vx<op_max_vx)
op_max_vx=temp_vx;
if(last_x_diff!=0&&((last_x_diff > 0 && xdiff< 0) || (last_x_diff < 0 &&  xdiff > 0) ) )
{flag=false;xdiff=last_x_diff;}
if(xdiff>30)
xdiff=0;
else if(xdiff>0.8){
xdiff=max_vx;
his_max_vx=xdiff;
}
else if(xdiff<-0.8){
xdiff=op_max_vx;
his_max_vx=xdiff;   
}
else 
xdiff=his_max_vx;

if(last_point.x!=0&&ydiff!=0)
    {
    last_x_diff=xdiff;
    last_y_diff=ydiff;
    } 



 klaman.statePost=(cv::Mat_<float>(4,1)<<real_point.x,real_point.y,xdiff,ydiff);
 cv::Mat prediction=klaman.predict();
 k++; 
// cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
 randn( measurement, cv::Scalar::all(0), cv::Scalar::all(klaman.measurementNoiseCov.at<float>(0))); 
 randn( processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(klaman.processNoiseCov.at<float>(0, 0))));
 klaman.statePost=klaman.statePost+processNoise;
 measurement += klaman.measurementMatrix*klaman.statePost;
 klaman.correct(measurement);
     if(last_point.x==0&&miss_detection_cnt>=5)
     {   cv::KalmanFilter cf(4,2);
        klaman=cf;//his_last_point.x=0.0;his_last_point.y=0.0;
        setIdentity(klaman.errorCovPost, cv::Scalar::all(1));
       // k=0;
        flag=false;}


cv::Point2f new_point
(relative_point.x+(prediction.at<float>(0)-real_point.x),//
 relative_point.y+(prediction.at<float>(1)-real_point.y));//
 cv::Point2f new_use_point
(prediction.at<float>(0)+6*(prediction.at<float>(0)-relative_point.x),//
prediction.at<float>(1)+(prediction.at<float>(1)-relative_point.y));//
if(abs(real_rect.angle-last_angle)>=10&&abs(last_point.x-real_point.x)/real_rect.size.width>=2.5)
{
    check_rota_cnt=25;
}
if(check_rota_cnt>0)
    {
        new_use_point.x=x_mid;
        new_use_point.y=real_point.y;
    }
if(check_rota_cnt>0)
check_rota_cnt--;
if(cal_two_point_dis(new_point,relative_point)>=1.5*armor_bf_predict.rect.size.width||
   cal_two_point_dis(new_point,relative_point)>=1.*armor_bf_predict.rect.size.height)
   flag=false;
   armor_predict.rect=cv::RotatedRect(new_use_point,armor_bf_predict.rect.size,armor_bf_predict.rect.angle);
   armor_predict.state=armor_bf_predict.state;

    
 
}
vision_mul::armor_pos Armor_recorder::SlectFinalArmor(std::vector<vision_mul::armor_info> &armors, AngleSolver& angle_slover, AngleSolverFactory& angle_slover_factory, cv::Mat & src) 
{
    std::vector<vision_mul::armor_pos> pos_vect;

    std::vector<vision_mul::armor_info> armor_vect;

    vision_mul::armor_pos armor_pos_;
    vision_mul::armor_pos armor_pos_predict;
    for (auto armor : armors)
    {       if(tiaobian_cnt<0){tiaobian_cnt=5;last_armorinfo=armor;}
            
    if(cal_two_point_dis(armor.rect.center,last_armorinfo.rect.center)
    >=1.2*(armor.rect.size.height+armor.rect.size.width)&&tiaobian_cnt>0)
    {armor=last_armorinfo
    ;tiaobian_cnt--;}
        double armor_ratio = std::max(armor.rect.size.width, armor.rect.size.height) / 
							 std::min(armor.rect.size.width, armor.rect.size.height);
        cv::RotatedRect rect = armor.rect;
        if (armor_ratio < 4)
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_SAMLL_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                
                armor_vect.push_back(armor);
                
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        }
        else
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                
                armor_vect.push_back(armor);
                
                
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        } // if infantry or hero
    last_armorinfo=armor;
    } // for
    
    vision_mul::armor_pos last_pos;

    if (history_pos.size())
    {
        last_pos = history_pos.back();
    }
  bool check;      
   vision_mul::armor_info armor_after_pr;
   vision_mul::armor_pos armor_pos_after_pr;
    if(pos_vect.size())
    {
        double dis_min = 100000000;
        int idx = 0;
        for (int i = 0; i != pos_vect.size(); ++i)
        {
            double dis = pos_distance(pos_vect[i],last_pos);
            if (dis < dis_min)
            {
                dis_min = dis;
                idx = i;
            }
        }
       // if(detect_cnt>=3)
       float rea_x_diff;
       float rea_y_diff;
        float suitz=this->get_midum_z();
        if(suitz!=0){
        rea_x_diff=0.15*0.01*suitz*current_yaw_v*976.5190/suitz;
        rea_y_diff=0.15*0.01*suitz*current_pitch_v*976.0805/suitz;}
        else 
        rea_x_diff=rea_y_diff=0;
        cv::RotatedRect real_rect_;
        kalman_predict(last_dst_point,km_, armor_vect[idx],armor_after_pr,check,rea_x_diff,rea_y_diff,real_rect_); 
        last_dst_point=real_rect_.center;
        last_angle=real_rect_.angle;
        //last_dst_point=armor_vect[idx].rect.center;
        draw_rotated_rect(src,real_rect_,cv::Scalar(255,255,0),2);
        draw_rotated_rect(src, armor_vect[idx].rect,cv::Scalar(0,255,255),2);
       
        if (check!=false){
        draw_rotated_rect(src, armor_after_pr.rect,cv::Scalar(255,0,255),2);
       
#ifdef use_predict            
        double armor_ratio = std::max(armor_after_pr.rect.size.width, armor_after_pr.rect.size.height) / 
							 std::min(armor_after_pr.rect.size.width, armor_after_pr.rect.size.height);
        cv::RotatedRect rect = armor_after_pr.rect;
        if (armor_ratio < 4)
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_SAMLL_ARMOR,armor_pos_after_pr.angle_x, armor_pos_after_pr.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_after_pr.Flag = armor_after_pr.state; // [1 2 3 4]
                armor_pos_after_pr.angle_z = angle_slover._distance;
                
                
               
                
            }
            else{
                armor_pos_after_pr.Flag = 0;
                armor_pos_after_pr.angle_z = angle_slover._distance;
            }
        }
        else
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_ARMOR, armor_pos_after_pr.angle_x, armor_pos_after_pr.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_after_pr.Flag = armor_after_pr.state; // [1 2 3 4]
                armor_pos_after_pr.angle_z = angle_slover._distance;
                
                
                
                
                
            }
            else{
                armor_pos_after_pr.Flag = 0;
                armor_pos_after_pr.angle_z = angle_slover._distance;
            }
        }
        printf("\n===========解算预测角度==========\n");
        return armor_pos_after_pr;
        #else 
        return pos_vect[idx];
        #endif
        }
        else return pos_vect[idx];
        
        
        
       
       
    }
    
     //detect_cnt=0;
    
    last_dst_point=cv::Point2f(0.0,0.0);
    return vision_mul::armor_pos();
}

} // namespace slover
} // namepsace autocar
