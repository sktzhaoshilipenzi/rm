/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy  of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#pragma once
#include <list>
#include <chrono>
#include "detect_factory/armor_info.h"
#include "slover/angle_slover.hpp"
#include"math.h"
namespace autocar
{
namespace slover
{ 
/**
 * @brief 记录历史信息,然后进行弹道预测.
 */
class Armor_recorder {
public:
    Armor_recorder(int _history_size = 5): history_size(_history_size) 
    { 
        recorder_time = std::chrono::steady_clock::now();
       std::vector<float> Pre_vels_(9);
       list_vx=Pre_vels_;
    }
    void   set_kalman_seed(cv::KalmanFilter km){km_=km;};
    double point_dist(const cv::Point2f & p1, const cv::Point2f & p2){ return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)); }
    void kalman_predict(cv::Point2f &last_point,cv::KalmanFilter &klaman,vision_mul::armor_info &armor_bf_predict,vision_mul::armor_info &armor_predict,bool &flag);
    vision_mul::armor_pos SlectFinalArmor(std::vector<vision_mul::armor_info> &armors, AngleSolver& angle_slover, AngleSolverFactory& angle_slover_factory,cv::Mat&);
    void setLastResult(/*vision_mul::armor_info last_armor,*/ vision_mul::armor_pos last_pos, double time)
    {
        if (history_pos.size() < history_size)
        {
            // history_armor.push_back(last_armor);
            history_pos.push_back(last_pos);
            history_time.push_back(time);
        }
        else {
            // history_armor.push_back(last_armor);
            history_pos.push_back(last_pos);           
            history_time.push_back(time);

            // history_armor.pop_front();
            history_pos.pop_front();
            history_time.pop_front();
        }
        recorder_time = std::chrono::steady_clock::now();
    }
    float PreFilter(float Pre_vel);
    void clear(){
        //history_armor.clear();
        //history_time.clear();
        if (/*history_armor.size() &&*/ history_pos.size() && history_time.size())
        {
            // history_armor.pop_front();
            history_pos.pop_front();
            history_time.pop_front();
        }
        
    }
    float cal_two_point_dis(cv::Point2f point1,cv::Point2f point2)
    {return  sqrt((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y));}
    std::chrono::steady_clock::time_point recorder_time;

    // std::list<vision_mul::armor_info> history_armor;
    std::list<vision_mul::armor_pos> history_pos;
    
    std::list<double> history_time;
    int history_size;
    int k;//滤波初始化检测因子
    int detect_cnt;
    int miss_detection_cnt;
    float last_x_diff;
    float last_y_diff;
    cv::KalmanFilter km_;
    double predict(double time);
    cv::Point2f last_dst_point;
    cv::Point2f his_last_point;
    std::vector<float> list_vx;
    std::vector<float> list_vx_copy;
    float max_vx;
    float op_max_vx;
    float his_max_vx;
   
};

} // namespace slover
} // namespace autocar
