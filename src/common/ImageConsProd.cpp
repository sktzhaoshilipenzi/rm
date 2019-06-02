/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
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

#include <chrono>
#include <vector>
#include "common/ImageConsProd.hpp"
#include "driver/camera_driver.h"
#include "driver/RMVideoCapture.hpp"
#include "detect_factory/armor_detect.hpp"

#include "slover/angle_slover.hpp"
#include "slover/armor_recorder.hpp"

#include "utility/video_recoder.h"
#include "utility/debug_utility.hpp"
#define BUFFER_SIZE 3

 //#define USE_VIDEO

namespace autocar
{
namespace vision_mul
{
volatile unsigned int prdIdx = 0;
volatile unsigned int csmIdx = 0;

struct ImageData {
	cv::Mat img;             // data come from camera
	unsigned int frame;      // 记数
};

ImageData capturedata[BUFFER_SIZE];   // Buffer of capture
void ImageConsProd:: ImageProductor()
{

    RMVideoCapture cap(usb_cam_id.c_str(), 3); 
	cap.setVideoFormat(1280, 720, 1);
	cap.startStream();
	cap.setExposureTime(false, exposure_time);

	while (true) {
		while (prdIdx - csmIdx >= BUFFER_SIZE);
		cap >> capturedata[prdIdx % BUFFER_SIZE].img;
		capturedata[prdIdx % BUFFER_SIZE].frame = cap.getFrameCount();
		++prdIdx;
	}
}
void ImageConsProd::ImageConsumer() 
{   
    driver::camera_driver camera_info(param_file);
#ifndef USE_VIDEO
    cv::VideoCapture capture_camera_forward;
    //camera_info.open_camera(capture_camera_forward, param_file);
#else
    cv::VideoCapture capture_camera_forward(video_name);
#endif
    camera_info.set_camera_pose();
    
    AngleSolver angle_slover(camera_info.camera_matrix, camera_info.dist_coeffs, 21.6, 5.4, camera_info.z_scale);
    Point2f image_center = Point2f(camera_info.camera_matrix.at<double>(0,2), camera_info.camera_matrix.at<double>(1,2));
    angle_slover.setRelationPoseCameraPTZ(camera_info.r_camera_ptz, camera_info.t_camera_ptz, camera_info.barrel_ptz_offset_y);
    AngleSolverFactory angle_slover_factory;
    angle_slover_factory.setTargetSize(21.6, 5.4, AngleSolverFactory::TARGET_ARMOR);
    angle_slover_factory.setTargetSize(12.4, 5.4, AngleSolverFactory::TARGET_SAMLL_ARMOR);
    angle_slover_factory.setSolver(&angle_slover);

    // 视频录制(如果打开视频，就不要视频录制)
#ifndef USE_VIDEO
    video_recoder recoder("../video", 1280, 720);
#endif
    cv::Mat frame_forward;
    cv::Mat frame_forward_src;
    ArmorDetector armor_detector;
    slover::Armor_recorder armor_recorder;
    KalmanFilter kem(4,2);
   // armor_recorder.set_kalman_seed(kem);
    setIdentity(kem.errorCovPost, Scalar::all(1));
    //km cc;
    std::vector<armor_info> multi_armors;

    while (true) {
        // some value reset
        speed_test_begin();
        
        armor_pos_.reset_pos();
        multi_armors.clear();
        
        // start camera
#ifdef USE_VIDEO
        capture_camera_forward >> frame_forward;
#else
        while (prdIdx - csmIdx == 0);
		capturedata[csmIdx % BUFFER_SIZE].img.copyTo(frame_forward);
		++csmIdx;
#endif
        if (save_result) frame_forward.copyTo(frame_forward_src);
       
        short current_yaw = serial_mul::get_yaw();
        short current_pitch = serial_mul::get_pitch();
        short pitch_v= serial_mul::get_pitchv();
        short yaw_v=serial_mul::get_yawv();
       // armor_recorder.set_current_pitch_v(pitch_v);
       // armor_recorder.set_current_yaw_v(yaw_v);
        if(armor_recorder.tiaobian_cnt!=5){armor_recorder.tiaobian_cnt--;}
        //std::cout << "vision_mul: " << current_yaw << ", " << current_pitch << std::endl;
        //angle_slover.set_current_pitch(current_pitch);
      //speed_test_reset();
       // speed_test_reset();
        if (armor_detector.detect(frame_forward, multi_armors)
            && multi_armors.size())
        {
            armor_pos_ = armor_recorder.SlectFinalArmor(multi_armors, angle_slover, angle_slover_factory,frame_forward);

            serial_mul::publish2car(armor_pos_, current_yaw, current_pitch);  // 发布消息
        }
        else{
            armor_pos_.reset_pos();
            serial_mul::publish2car(armor_pos_, current_yaw, current_pitch); 
        }
        // speed_test_end("det time cost = ", "ms");
        /*float tm=armor_recorder.history_time.back();
        Vec3f jiaodu;
        jiaodu[0]=armor_pos_.angle_x;
        jiaodu[1]=armor_pos_.angle_y;
        jiaodu[2]=armor_pos_.angle_z;
        Vec3f shangyizhen;
        shangyizhen[0]=armor_recorder.history_pos.back().angle_x;
        shangyizhen[1]=armor_recorder.history_pos.back().angle_y;
        shangyizhen[2]=armor_recorder.history_pos.back().angle_z;
        Vec3f yucedian=cc.predict(kem,shangyizhen,jiaodu);
         fprintf(pl,"实际点%f %f %f 预测点%f %f %f\n",jiaodu[0],jiaodu[1],jiaodu[2],yucedian[0],yucedian[1],yucedian[2]);*/
        if (armor_pos_.Flag)
        {
            armor_recorder.miss_detection_cnt = 0;
            armor_recorder.setLastResult(armor_pos_, record_time);
     //       armor_recorder.preFilter_for_z(armor_pos_.angle_z);
        }
        else {
            armor_recorder.miss_detection_cnt++;
            if (armor_recorder.miss_detection_cnt > 3) armor_recorder.clear();
        }
        /*fprintf(pl,"前五次的角度");
        for(const auto& hh : armor_recorder.history_pos)
        {
              // printf("%f  %f %f\n",hh.angle_x,hh.angle_y,hh.angle_z);
               fprintf(pl,"%f %f %f ",hh.angle_x,hh.angle_y,hh.angle_z);

        }
        fprintf(pl,"\n");*/
       
		if (show_image > 0)
        {  // Point2f dst_point = Point2f(camera_info.camera_matrix.at<double>(0,2)+10*armor_pos_.angle_x, camera_info.camera_matrix.at<double>(1,2)+10*armor_pos_.angle_y);
            // show center and result
            circle(frame_forward, image_center, 3, CV_RGB(0, 255, 0), 2);  // 在图像中心画一个准心
          //  circle(frame_forward, dst_point, 3, CV_RGB(255, 0, 0), 2);
            char str[30];
            sprintf(str, "%.2f, %.2f, %.2fcm ,%d flag", armor_pos_.angle_x, armor_pos_.angle_y, armor_pos_.angle_z,armor_pos_.Flag);
            putText(frame_forward, str, Point(5, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(100,205,0), 1);
           // speed_test_reset();
            try {
                //cvtColor(frame_forward, frame_forward,CV_RGB2GRAY);
				imshow("result", frame_forward);
                cv::waitKey(1); // 很有必要
			}
			catch (cv::Exception e) {
				std::cout << "show image error" << std::endl;
			}
        }
        speed_test_end("total time cost = ", "ms");
#ifdef USE_VIDEO
       cv::waitKey(0);  // 暂停用来调试
#else
        if(save_result)  // 使用视频不录像
            recoder.save_frame(frame_forward_src); // 录制
#endif
	}   // end while loop
}

} // namespace vision_mul
} // namespace autocar

