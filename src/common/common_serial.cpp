/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <chrono>
#include <thread> 
#include "common/common_serial.h"
#include <cstring>
namespace autocar
{
namespace serial_mul
{

#define show_serial_listen
#define show_serial_publish

SerialDevice serial("/dev/robomaster", 115200);
short Yaw    = 0;
short Pitch  = 0;
short yaw_v  = 0;
short pitch_v= 0;
short get_yaw() 
{
    short a = Yaw;
    return a;
}
short get_pitch() 
{
    short b = Pitch;
    return b;
}
short get_pitchv()
{
    short a=pitch_v;
    float b=float(a);
    
    return a;
    
}
short get_yawv()
{
    short b=yaw_v;
    float a=float(b);
   
    return b;
}
void listen2car()
{
    while(1)
    {  
        unsigned char buffread_fix[11];
        unsigned char buffread[11];
        unsigned char data[11];// = {0xDA,
                              //   0x00,0x00,  // Yaw
                              //   0x00,0x00,  // Pitch
                              //   0xDB};
        serial.Read(buffread, 11);
        int len=sizeof(data);
        for(int i = 0; i < len; ++i)
      {
        if(buffread[i] == 0xDA)
        {
          memcpy(buffread_fix, buffread + i, len - i);
          memcpy(buffread_fix + len - i, buffread, i);
          memcpy(&data, buffread_fix, len);        
        }
      }
        // 这里可能需要一个标志位,告诉我旋转...
        if (data[0] == 0xDA && data[10] == 0xDB)
        {
            Yaw   = (data[1]<<8) + data[2];
            Pitch = (data[3]<<8) + data[4];
            yaw_v = (data[5]<<8)+data[6];
            pitch_v=(data[7]<<8)+data[8];
        }
#ifdef show_serial_listen
        std::cout << "陀螺仪\tYaw: "<< Yaw/100.0 << "\tPitch: "<< Pitch/100.0 << std::endl;
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void publish2car(const vision_mul::armor_pos& pos, short _yaw, short _pitch)
{
    unsigned char send_bytes[] = { 0xFF,        // 头
                                   0x00,        // BOOL
                                   0x00,0x00,   // Yaw
                                   0x00,0x00,   // Pitch
                                   0x00,        // 距离
                                   0xFE};       // 尾
    send_bytes[1] = pos.Flag;     // 标志位
    send_bytes[6] = 0.1*pos.angle_z;  // 距离信息
    
    short* data_ptr = (short *)(send_bytes + 2); // 16位指针指向第一个数据
    
    data_ptr[0] = _yaw   + static_cast<short>((pos.angle_x) * 100);
    data_ptr[1] = _pitch - static_cast<short>((pos.angle_y) * 100);

    serial.Write(send_bytes, 8);
#ifdef show_serial_publish
    std::cout << "send_data...\t" << pos.Flag  << "\t\t"
              << Yaw/100.0 <<" - " << pos.angle_x << "\t\t"
              << Pitch/100.0 <<" - " << pos.angle_y << "\t\t"
              << pos.angle_z << std::endl;
#endif
}
            

} // namespace serial_mul
} // namespace autocar