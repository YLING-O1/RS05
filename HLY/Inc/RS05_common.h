//
// Created by 34285 on 2026/5/10.
//

#ifndef RS05_RS05_COMMON_H
#define RS05_RS05_COMMON_H

#include "stm32f4xx_hal.h"

//实际电机数量
#define MAX_MOTOR_COUNT 1

//自定义8位CAN数据包
//滤波器编号
#define CAN_FILTER(x) ((x) << 3)
//接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)
//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)
//数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

//配置滤波器中的分配过滤器的参数
#define  FILTER_BANK 14

//浮点数与无符号整数转换联合体
typedef union
{
    float f;
    uint32_t u;
    uint8_t b[4];
}Float_Uint_TypeDef;

//RS05电机协议
typedef enum
{
    RS05_PROTOCOL_PRIVATE, // 私有协议 (29位扩展帧)
    RS05_PROTOCOL_CANOPEN, // CANopen协议
    RS05_PROTOCOL_MIT      // MIT协议 (11位标准帧)
}Motor_Protocol;

//RS05电机面向对象结构体
typedef struct
{
    //写参数（发送给电机的参数）————————————————————————————————————————————————————————————————————————————————————————————
    //私有协议：目标角度 [0~65535] 对应 (-4PI~4PI)；MIT协议:目标角度 [0~65535] 对应 (-12.57rad~12.57rad)
    float Target_Angle;
    //目标角速度 [0~65535] 对应 (-50rad/s~50rad/s)
    float Target_Omega;
    //发送的Kp [0~65535] 对应 (0.0~500.0)
    float Send_Kp;
    //发送的Kd [0~65535] 对应 (0.0~5.0)
    float Send_Kd;
    //MIT协议:目标扭矩 [0~65535] 对应 (-5.5Nm~5.5Nm)
    float Target_Torque;
    //主机发送用的 CAN ID
    uint8_t HOST_CAN_ID;
    //目标电机的 CAN ID
    uint8_t MOTOR_CAN_ID;
    //发送的数据帧的8字节数据包
    uint8_t CAN_Tx_Data[8];

    //读参数（电机反馈的参数）——————————————————————————————————————————————————————————————————————————————————————————————
    //当前角度 [0~65535] 对应 (-4PI~4PI)；MIT协议:当前角度 [0~65535] 对应 (-12.57rad~12.57rad)
    uint16_t Current_Angle;
    //当前角速度 [0~65535] 对应 (-50rad/s~50rad/s)；MIT协议:当前角速度 [0~4096] 对应 (-50rad/s~50rad/s)
    uint16_t Current_Omega;
    //私有协议：当前扭矩 [0~65535] 对应 (-5.5Nm~5.5Nm)；MIT协议:当前扭矩 [0~4096] 对应 (-5.5Nm~5.5Nm)
    uint16_t Current_Torque;
    //当前温度: 温度(摄氏度)用两个字节表示，高字节在先、低字节在后,发送或读取的温度数值是实际摄氏温度乘以 10 之后的值
    float Current_Temperature;
    //私有协议：当前电机CANID
    uint16_t Target_Motor_CAN_ID;
    //MIT协议:当前电机的CANID
    uint8_t Motor_CAN_ID;
    //故障信息0无1有
    uint8_t Fault_Information;
    //电机堵转过载故障
    uint8_t Gridlock_Overload_Fault;
    //磁性编码故障
    uint8_t Magnetic_Coding_Fault;
    //过热
    uint8_t Overtemperature;
    //三相过流故障
    uint8_t Three_Phase_Overcurrent_Fault;
    //欠压故障,在通信类型21中占位2
    uint8_t Undervoltage_Fault;
    //模式状态，0为复位模式，1为校准模式，2为电机运行模式
    uint8_t Mode_Status;
    //主机CANID
    uint8_t Master_CAN_ID;
    //是否成功读取主机ID，00成功，01失败
    uint8_t Master_ID_Is_Read;
    //参数数据，低字节在前，高字节在后
    uint32_t Parameter_Data;
    //故障值
    uint32_t Fault_Value;
    //位16，A相采样过流
    uint8_t A_Phase_Overcurrent_Sampling;
    //位14，电机失速过载算法保护
    uint8_t Motor_Stall_Overload_Algorithm_Protection;
    //位9，位置初始化错误
    uint8_t Position_Initialization_Fault;
    //位8，硬件识别故障
    uint8_t Hardware_Identification_Fault;
    //位7，编码器未校准
    uint8_t Encoder_Uncalibrated;
    //位5，C相采样过流
    uint8_t C_Phase_Overcurrent_Sampling;
    //位4，B相采样过流
    uint8_t B_Phase_Overcurrent_Sampling;
    //位3，过压故障
    uint8_t Overvoltage_Fault;
    //位1，驱动芯片故障
    uint8_t Driver_Chip_Fault;
    //位0，电机过热故障
    uint8_t Motor_Overtemperature_Fault;
    //警告值
    uint32_t Warning_Value;
    //位0，电机过温警告
    uint8_t Motor_Overtemperature_Warning;
    //参数表索引
    uint16_t Index;
    //参数数据
    uint32_t Parameter;
}Motor_Manage_Object;

//CAN接收的信息结构体
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Rx_Data[8];
}Struct_CAN_Rx_Buffer;

//CAN接收回调函数数据类型
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer*, Motor_Manage_Object*);

//CAN处理接收到的信息的结构体
typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;
    CAN_Call_Back CallBack_Function;

    Motor_Manage_Object* Motor_List[MAX_MOTOR_COUNT];
    uint8_t Motor_Count;
}Struct_CAN_Manage_Object;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

void CAN_Init(CAN_HandleTypeDef* hcan,CAN_Call_Back CallBack_Function);
void Motor_Register(Struct_CAN_Manage_Object* CAN_Manage_Object, Motor_Manage_Object* Motor_Object);

#endif //RS05_RS05_COMMON_H
