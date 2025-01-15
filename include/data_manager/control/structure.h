#ifndef CONTROL_STRUCTURE_H_
#define CONTROL_STRUCTURE_H_
#include <cstdint>

struct FrameHeader {
    uint8_t     sof;
    uint8_t     crc8;
} __attribute__((packed));

struct FrameTailer {
    uint16_t    crc16;
} __attribute__((packed));


struct InputData {
    uint8_t     enemy_color;
} __attribute__((packed));

struct OutputData {
    float       x00;
    float       x01;
    float       x02;
    float       x03;
    float       x10;
    float       x11;
    float       x12;
    float       x13;
    float       x20;
    float       x21;
    float       x22;
    float       x23;
    float       x30;
    float       x31;
    float       x32;
    float       x33;
} __attribute__((packed));



struct StateBytes {                     // 电控传给自瞄系统的云台数据
    FrameHeader frame_header;
    InputData   input_data;
    FrameTailer frame_tailer;
} __attribute__((packed));

struct OperateBytes {                   // 自瞄返回给电控的控制数据
    FrameHeader frame_header;
    OutputData  output_data;
    FrameTailer frame_tailer;
} __attribute__((packed));


#endif