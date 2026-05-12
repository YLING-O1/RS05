//
// Created by 34285 on 2026/5/9.
//

#include "MIT_RS05.h"

//CAN发送缓冲区——————————————————————————————————————————————————————————————————————————————————————————————————————————
//命令1：使能电机操作的8位数据包打包
void MIT_CAN_Tx_Data_Enable_Motor_Operation_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFC;
}

//命令2：停止电机操作的8位数据包打包
void MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFD;
}

//命令3：MIT动态参数的8位数据包打包
void MIT_CAN_Tx_Data_MIT_Dynamic_Parameters_Packet(float Target_Angle, float Target_Omega,
                                               float Send_Kp, float Send_Kd, float Target_Torque,
                                               Motor_Manage_Object *RS05_Manage_Object)
{
    uint16_t angle_uint  = float_to_uint16 (Target_Angle, -4 * PI, 4 * PI, 12);
    uint16_t omega_uint  = float_to_uint16 (Target_Omega, -50.0f, 50.0f, 12);
    uint16_t kp_uint     = float_to_uint16 ( Send_Kp, 0.0f, 500.0f, 12);
    uint16_t kd_uint     = float_to_uint16 ( Send_Kd, 0.0f, 100.0f, 12);
    uint16_t torque_uint = float_to_uint16 (Target_Torque, -5.5f, 5.5f, 12);

    //大端模式
    RS05_Manage_Object->CAN_Tx_Data[0] = (angle_uint & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[1] = angle_uint & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = (omega_uint & 0xFF0) >> 4;
    RS05_Manage_Object->CAN_Tx_Data[3] = ((omega_uint & 0xF) << 4) | ((kp_uint & 0xF00) >> 8);
    RS05_Manage_Object->CAN_Tx_Data[4] = kp_uint & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = (kd_uint & 0xFF0) >> 4;
    RS05_Manage_Object->CAN_Tx_Data[6] = ((kd_uint & 0xF) << 4) | ((torque_uint & 0xF00) >> 8);
    RS05_Manage_Object->CAN_Tx_Data[7] = torque_uint & 0xFF;
}

//命令4：设置零点（非位置模式）的8位数据包打包
void MIT_CAN_Tx_Data_Set_Zero_Position_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFE;
}

//命令5：清处错误并读取故障状态的8位数据包打包,清除电机报错时，F_CMD置0xFF；查询电机当前的故障代码时，F_CMD置0x00
void MIT_CAN_Tx_Data_Clear_Errors_And_Read_Fault_Status_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFB;
}

//命令6：设定操作模式的8位数据包打包，F_CMD为操作模式，0为运动控制模式（默认模式），1为位置模式，2为速度模式
void MIT_CAN_Tx_Data_Set_Operation_Mode_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFC;
}

//命令7：修改电机CANID的8位数据包打包，F_CMD为电机预设ID
void MIT_CAN_Tx_Data_Modify_Motor_CAN_ID_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFA;
}

//命令8：更改通信协议（重新上电生效）的8位数据包打包，F_CMD为电机协议类型，0为私有协议（默认），1为CANopen协议，2为MIT协议
void MIT_CAN_Tx_Data_Change_Communication_Protocol_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xFD;
}

//命令9：修改主机CANID的8位数据包打包，F_CMD为主机预设ID
void MIT_CAN_Tx_Data_Modify_Host_CAN_ID_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x01;
}

//命令10：位置模式控制命令的8位数据包打包,参数为目标位置（rad），目标速度（rad/s）
void MIT_CAN_Tx_Data_Position_Mode_Control_Packet(float Target_Position, float Target_Speed,
                                              Motor_Manage_Object *RS05_Manage_Object)
{
    Float_Uint_TypeDef Position = {.f = Target_Position};
    Float_Uint_TypeDef Speed = {.f = Target_Speed};

    RS05_Manage_Object->CAN_Tx_Data[0] = (Position.u & 0xFF000000) >> 24;
    RS05_Manage_Object->CAN_Tx_Data[1] = (Position.u & 0xFF0000) >> 16;
    RS05_Manage_Object->CAN_Tx_Data[2] = (Position.u & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[3] = (Position.u & 0xFF);
    RS05_Manage_Object->CAN_Tx_Data[4] = (Speed.u & 0xFF000000) >> 24;
    RS05_Manage_Object->CAN_Tx_Data[5] = (Speed.u & 0xFF0000) >> 16;
    RS05_Manage_Object->CAN_Tx_Data[6] = (Speed.u & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[7] = (Speed.u & 0xFF);
}

//命令11：速度模式控制命令的8位数据包打包参数为目标速度（rad/s），电流限制（A）
void MIT_CAN_Tx_Data_Velocity_Mode_Control_Packet(float Target_Speed, float Current_Limit,
                                              Motor_Manage_Object *RS05_Manage_Object )
{
    Float_Uint_TypeDef Speed = {.f = Target_Speed};
    Float_Uint_TypeDef Limit = {.f = Current_Limit};

    RS05_Manage_Object->CAN_Tx_Data[0] = (Speed.u & 0xFF000000) >> 24;
    RS05_Manage_Object->CAN_Tx_Data[1] = (Speed.u & 0xFF0000) >> 16;
    RS05_Manage_Object->CAN_Tx_Data[2] = (Speed.u & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[3] = (Speed.u & 0xFF);
    RS05_Manage_Object->CAN_Tx_Data[4] = (Limit.u & 0xFF000000) >> 24;
    RS05_Manage_Object->CAN_Tx_Data[5] = (Limit.u & 0xFF0000) >> 16;
    RS05_Manage_Object->CAN_Tx_Data[6] = (Limit.u & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[7] = (Limit.u & 0xFF);
}

//命令12：电机数据保存
void MIT_CAN_Tx_Data_Motor_Data_Save_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xF8;
}

//命令13：电机主动上报,其中 F_CMD 字节为电机协议类型,其中 0 为不上报（默认）,1为上报
void MIT_CAN_Tx_Data_Motor_Active_Report_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0xF9;
}

//指令 14：电机读参数的8位数据包打包，参数为要读取的参数索引
void MIT_CAN_Tx_Data_Motor_Parameter_Read_Packet(uint16_t index, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = index & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = (index & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x0;
}

//指令 15：电机写参数的8位数据包打包，参数为要读取的参数索引
void MIT_CAN_Tx_Data_Motor_Parameter_Write_Packet(uint16_t index, uint32_t parameter_data,
                                                  Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = index & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[1] = (index & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x0;
    RS05_Manage_Object->CAN_Tx_Data[4] = parameter_data & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[5] = (parameter_data & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[6] = (parameter_data & 0xFF0000) >> 16;
    RS05_Manage_Object->CAN_Tx_Data[7] = (parameter_data & 0xFF000000) >> 24;
}


//配置筛选器为标准数据帧32位————————————————————————————————————————————————————————————————————————————————————————————————
void MIT_CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID)
{
    //自定义滤波器结构体名称
    CAN_FilterTypeDef MIT_can_filter_mask_structure;

    //通过第0位ID，判断是数据帧或遥控帧，遥控帧不处理
    if (Object_Para & 0x01)
    {
        return;
    }

    MIT_can_filter_mask_structure.FilterIdHigh = (Filter_ID & 0x7FF) << 5;
    MIT_can_filter_mask_structure.FilterIdLow = 0x00;
    MIT_can_filter_mask_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
    MIT_can_filter_mask_structure.FilterMaskIdLow = 0x00;
    MIT_can_filter_mask_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    MIT_can_filter_mask_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    MIT_can_filter_mask_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    MIT_can_filter_mask_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    MIT_can_filter_mask_structure.FilterActivation = ENABLE;
    MIT_can_filter_mask_structure.SlaveStartFilterBank = FILTER_BANK;

    HAL_CAN_ConfigFilter(hcan, &MIT_can_filter_mask_structure);
}

//发送数据帧11位标识符计算—————————————————————————————————————————————————————————————————————————————————————————————————
//命令10：位置模式控制
uint16_t MIT_Position_Mode_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return (0x1 << 8) | RS05_Manage_Object->MOTOR_CAN_ID;
}

//命令11：速度模式控制
uint16_t MIT_Velocity_Mode_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return (0x2 << 8) | RS05_Manage_Object->MOTOR_CAN_ID;
}

//指令14：电机读参数
uint16_t MIT_Motor_Parameter_Read_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return (0x3 << 8) | RS05_Manage_Object->MOTOR_CAN_ID;
}

//指令 15：电机写参数
uint16_t MIT_Motor_Parameter_Write_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return (0x4 << 8) | RS05_Manage_Object->MOTOR_CAN_ID;
}


//发送数据帧—————————————————————————————————————————————————————————————————————————————————————————————————————————————
uint8_t MIT_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint16_t CAN_ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = CAN_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t Tx_Mailbox;
    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &Tx_Mailbox);
}

//RS05电机MIT模式下的反馈帧解析————————————————————————————————————————————————————————————————————————————————————————————
void MIT_CAN_RS05_Call_Back(Struct_CAN_Rx_Buffer* CAN_Rx_Buffer, Motor_Manage_Object* RS05_Manage_Object)
{
    //switch不支持任何形式的变量运算或指针寻址,switch-case语句的case标签必须是在编译时就能确定的整型常量
    //MIT模式的反馈帧类型只有两种，所以可以用else if链
    uint16_t ID = CAN_Rx_Buffer->Header.StdId;
    //响应命令1：数据反馈电机状态
    if (ID == RS05_Manage_Object->HOST_CAN_ID)
    {
        if (CAN_Rx_Buffer->Rx_Data[0] != RS05_Manage_Object->MOTOR_CAN_ID)
        {
            //如果反馈帧的电机ID与当前管理对象的电机ID不匹配，说明这条反馈帧不是针对当前管理对象的，直接丢弃
            return;
        }

        RS05_Manage_Object->Motor_CAN_ID = CAN_Rx_Buffer->Rx_Data[0];
        RS05_Manage_Object->Current_Angle = (CAN_Rx_Buffer->Rx_Data[1] << 8) | CAN_Rx_Buffer->Rx_Data[2];
        RS05_Manage_Object->Current_Omega = (CAN_Rx_Buffer->Rx_Data[3] << 4) |
                                            ((CAN_Rx_Buffer->Rx_Data[4] & 0xF0) >> 4);
        RS05_Manage_Object->Current_Torque = ((CAN_Rx_Buffer->Rx_Data[4] & 0x0F) << 8) |
                                             CAN_Rx_Buffer->Rx_Data[5];
        RS05_Manage_Object->Current_Temperature = (float)((int16_t)((CAN_Rx_Buffer->Rx_Data[6] << 8) |
                                                  CAN_Rx_Buffer->Rx_Data[7])) / 10.0f;
    }
    //响应命令2:MCU识别
    // else if (ID == RS05_Manage_Object->MOTOR_CAN_ID)
    // {
    //     //8字节数据包都为64位MCU唯一标识符，暂不处理
    // }
    //电机读参数的应答帧
    else if (ID == ((0x3 << 8) | RS05_Manage_Object->MOTOR_CAN_ID))
    {
        RS05_Manage_Object->Index = CAN_Rx_Buffer->Rx_Data[0] | (CAN_Rx_Buffer->Rx_Data[1] << 8);
        RS05_Manage_Object->Parameter = CAN_Rx_Buffer->Rx_Data[4] | (CAN_Rx_Buffer->Rx_Data[5] << 8) |
                                             (CAN_Rx_Buffer->Rx_Data[6] << 16) | (CAN_Rx_Buffer->Rx_Data[7] << 24);
    }
    //电机写参数的应答帧
    else if (ID == ((0x4 << 8) | RS05_Manage_Object->MOTOR_CAN_ID))
    {
        RS05_Manage_Object->Index = CAN_Rx_Buffer->Rx_Data[0] | (CAN_Rx_Buffer->Rx_Data[1] << 8);
        RS05_Manage_Object->Parameter = CAN_Rx_Buffer->Rx_Data[4] | (CAN_Rx_Buffer->Rx_Data[5] << 8) |
                                             (CAN_Rx_Buffer->Rx_Data[6] << 16) | (CAN_Rx_Buffer->Rx_Data[7] << 24);
    }
}

//MIT控制模式———————————————————————————————————————————————————————————————————————————————————————————————————————————
//运控模式
void MIT_Motion_Control_Mode(CAN_HandleTypeDef* hcan, float Target_Angle, float Target_Omega,
                             float Send_Kp, float Send_Kd, float Target_Torque,
                             Motor_Manage_Object* RS05_Manage_Object)
{
    //发送电机使能指令（指令 1）
    uint16_t ID = RS05_Manage_Object->MOTOR_CAN_ID;
    MIT_CAN_Tx_Data_Enable_Motor_Operation_Packet(RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //发送运控指令（指令 3）以激活动态参数控制

    MIT_CAN_Tx_Data_MIT_Dynamic_Parameters_Packet(Target_Angle, Target_Omega, Send_Kp, Send_Kd, Target_Torque,
                                                  RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //在需要时，发送电机停止指令（指令 2）以停止运行
    //MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(RS05_Manage_Object);
    //MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//速度模式
void MIT_Velocity_Mode(CAN_HandleTypeDef* hcan, float Target_Speed, float Current_Limit,
                       Motor_Manage_Object* RS05_Manage_Object)
{
    //发送“设置运行模式指令”（指令 6）并设定模式 = 2（速度模式），来配置电机的运行模式
    uint16_t ID = RS05_Manage_Object->MOTOR_CAN_ID;
    MIT_CAN_Tx_Data_Set_Operation_Mode_Packet(0x02, RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //发送“电机使能指令”（指令 1）来激活电机
    MIT_CAN_Tx_Data_Enable_Motor_Operation_Packet(RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //发送“速度模式控制指令”（指令 11）来设置最大电流（绝对值）和目标速度
    ID = MIT_Velocity_Mode_Calculate_ID(RS05_Manage_Object);
    MIT_CAN_Tx_Data_Velocity_Mode_Control_Packet(Target_Speed, Current_Limit, RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //如需停止，发送“电机停止指令”（指令 2）
    //MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(RS05_Manage_Object);
    //MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//位置模式（CSP-循环同步位置）
void MIT_Position_Mode_CSP(CAN_HandleTypeDef* hcan, float Target_Position, float Target_Speed,
                           Motor_Manage_Object* RS05_Manage_Object)
{
    //发送“设置运行模式指令”（指令 6）并设定模式 = 1（位置模式），来配置电机的运行模式
    uint16_t ID = RS05_Manage_Object->MOTOR_CAN_ID;
    MIT_CAN_Tx_Data_Set_Operation_Mode_Packet(0x01, RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //发送“电机使能指令”（指令 1）来激活电机
    MIT_CAN_Tx_Data_Enable_Motor_Operation_Packet(RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //发送“位置模式控制指令”（指令 10）来设置最大速度（绝对值）和目标位置
    ID = MIT_Position_Mode_Calculate_ID(RS05_Manage_Object);
    MIT_CAN_Tx_Data_Position_Mode_Control_Packet(Target_Position, Target_Speed, RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //如需停止，发送“电机停止指令”（指令 2）
    //MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(RS05_Manage_Object);
    //MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//停止运行模式
void MIT_Stop_Mode(CAN_HandleTypeDef* hcan, Motor_Manage_Object* RS05_Manage_Object)
{
    //发送电机停止指令（指令 2）以停止运行
    uint16_t ID = RS05_Manage_Object->MOTOR_CAN_ID;
    MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(RS05_Manage_Object);
    MIT_CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}
