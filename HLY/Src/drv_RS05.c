//
// Created by 34285 on 2026/4/28.
//
#include "drv_RS05.h"


//CAN发送缓冲区——————————————————————————————————————————————————————————————————————————————————————————————————————————
//通信类型0：获取设备ID的8字节数据包打包
void CAN_Tx_Data_Get_Device_ID_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0;
}

//通信类型1：操作控制模式电机控制指令的8字节数据包打包
void CAN_Tx_Data_Operation_Control_Packet(float Target_Angle, float Target_Omega, float Send_Kp, float Send_Kd,
                                          Motor_Manage_Object *RS05_Manage_Object)
{
    uint16_t angle_uint  = float_to_uint16 (Target_Angle, -4 * PI, 4 * PI, 16);
    uint16_t omega_uint  = float_to_uint16 (Target_Omega, -50.0, 50.0, 16);
    uint16_t kp_uint     = float_to_uint16 ( Send_Kp, 0.0, 500.0, 16);
    uint16_t kd_uint     = float_to_uint16 ( Send_Kd, 0.0, 5.0, 16);

    //大端模式
    RS05_Manage_Object->CAN_Tx_Data[0] = (angle_uint & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[1] = angle_uint & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[2] = (omega_uint & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[3] = omega_uint & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[4] = (kp_uint & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[5] = kp_uint & 0xFF;
    RS05_Manage_Object->CAN_Tx_Data[6] = (kd_uint & 0xFF00) >> 8;
    RS05_Manage_Object->CAN_Tx_Data[7] = kd_uint & 0xFF;
}

//通信类型3：电机使能运行的8字节数据包打包
void CAN_Tx_Data_Motor_Enable_Run_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0;
}

//通信类型4：电机停止运行的8字节数据包打包；字节0置0时正常停止，字节0置1时清除故障,mode置0或1
void CAN_Tx_Data_Motor_Stop_Running_Packet(uint8_t mode, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = mode;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0;
}

//通信类型6：设置电机机械零位的8字节数据包打包
void CAN_Tx_Data_Set_Motor_Mechanical_Zero_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x1;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0;
}

//通信类型17：单参数读取的8字节数据包打包
void CAN_Tx_Data_Single_Parameter_Read_Packet(uint16_t index, Motor_Manage_Object *RS05_Manage_Object)
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

//通信类型18：单参数写入（掉电丢失）的8字节数据包打包
void CAN_Tx_Data_Single_Parameter_Write_Packet(uint16_t index, uint32_t parameter_data,
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

//通信类型22：电机数据保存帧的8字节数据包打包
void CAN_Tx_Data_Motor_Data_Save_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x01;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0x02;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x03;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x04;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0x05;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0x06;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0x07;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x08;
}

//通信类型23：电机波特率修改帧（重新上电生效）的8字节数据包打包,F_CMD中，01为1M，02为500K，03为250K，04为125K
void CAN_Tx_Data_Motor_Baud_Rate_Modification_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x01;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0x02;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x03;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x04;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0x05;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0x06;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x0;
}

//通信类型24：电机主动报告帧的8字节数据包打包,F_CMD中，00为禁止主动报告（默认），01为启动主动报告，默认报告间隔为10ms
void CAN_Tx_Data_Motor_Actively_Report_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x01;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0x02;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x03;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x04;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0x05;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0x06;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x0;
}

//通信类型25：电机协议修改帧（重新上电生效）的8字节数据包打包,F_CMD为电机协议类型，0为私有协议（默认），1为CANopen协议，2为MIT协议
void CAN_Tx_Data_Motor_Protocol_Modification_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x01;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0x02;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0x03;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0x04;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0x05;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0x06;
    RS05_Manage_Object->CAN_Tx_Data[6] = F_CMD;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0x0;
}

//通信类型26：版本号读取帧的8字节数据包打包
void CAN_Tx_Data_Version_Number_Read_Packet(Motor_Manage_Object *RS05_Manage_Object)
{
    RS05_Manage_Object->CAN_Tx_Data[0] = 0x00;
    RS05_Manage_Object->CAN_Tx_Data[1] = 0xC4;
    RS05_Manage_Object->CAN_Tx_Data[2] = 0;
    RS05_Manage_Object->CAN_Tx_Data[3] = 0;
    RS05_Manage_Object->CAN_Tx_Data[4] = 0;
    RS05_Manage_Object->CAN_Tx_Data[5] = 0;
    RS05_Manage_Object->CAN_Tx_Data[6] = 0;
    RS05_Manage_Object->CAN_Tx_Data[7] = 0;
}


//配置筛选器为扩展数据帧32位————————————————————————————————————————————————————————————————————————————————————————————————
void CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID)
{
    //自定义滤波器结构体名称
    CAN_FilterTypeDef can_filter_mask_structure;

    //通过第0位ID，判断是数据帧或遥控帧，遥控帧不处理
    if (Object_Para & 0x01)
    {
        return;
    }

    can_filter_mask_structure.FilterIdHigh = (Filter_ID & 0x1FFFFFFF) >> 13;
    can_filter_mask_structure.FilterIdLow = ((Filter_ID & 0x1FFF) << 3) | (1 << 2);
    can_filter_mask_structure.FilterMaskIdHigh = (Mask_ID & 0x1FFFFFFF) >> 13;
    can_filter_mask_structure.FilterMaskIdLow = ((Mask_ID & 0x1FFF) << 3) | (1 << 2);
    can_filter_mask_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    can_filter_mask_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    can_filter_mask_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_mask_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_mask_structure.FilterActivation = ENABLE;
    can_filter_mask_structure.SlaveStartFilterBank = FILTER_BANK;

    HAL_CAN_ConfigFilter(hcan, &can_filter_mask_structure);
}

//发送数据帧29位标识符计算—————————————————————————————————————————————————————————————————————————————————————————————————
//通信类型0：获取设备ID
uint32_t Get_Device_ID_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x0) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型1：操作控制模式电机控制指令
uint32_t Operation_Control_ID(uint16_t Target_Torque, Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x1) | CAN_ID_DATA_AREA2(Target_Torque) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型3：电机使能运行
uint32_t Motor_Enable_Run_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x3) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型4：电机停止运行
uint32_t Motor_Stop_Running_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x4) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型6：设置电机机械零位
uint32_t Set_Motor_Mechanical_Zero_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x6) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型7：设置电机CANID
uint32_t Set_Motor_CAN_ID_ID(uint8_t Preset_CAN_ID, Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x7) | CAN_ID_DATA_AREA2((RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           ((Preset_CAN_ID & 0xFF) << 8)) | CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型17：单参数读取
uint32_t Single_Parameter_Read_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x11) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型18：单参数写入（掉电丢失）
uint32_t Single_Parameter_Write_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x12) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型22：电机数据保存帧
uint32_t Motor_Data_Save_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x16) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型23：电机波特率修改帧（重新上电生效）
uint32_t Motor_Baud_Rate_Modification_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x17) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型24：电机主动报告帧
uint32_t Motor_Actively_Report_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x18) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型25：电机协议修改帧（重新上电生效）
uint32_t Motor_Protocol_Modification_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x19) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//通信类型26：版本号读取帧
uint32_t Version_Number_Read_ID(Motor_Manage_Object *RS05_Manage_Object)
{
    return CAN_ID_COMMUNICATION_TYPE(0x4) | CAN_ID_DATA_AREA2(RS05_Manage_Object->HOST_CAN_ID & 0xFF) |
           CAN_ID_DESTINATION_ADDRESS(RS05_Manage_Object->MOTOR_CAN_ID);
}

//发送数据帧
uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint32_t CAN_ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0;
    tx_header.ExtId = CAN_ID;
    tx_header.IDE = 1;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t Tx_Mailbox;
    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &Tx_Mailbox);
}

//RS05电机反馈帧解析
void CAN_RS05_Call_Back(Struct_CAN_Rx_Buffer* CAN_Rx_Buffer, Motor_Manage_Object* RS05_Manage_Object)
{
    //switch不支持任何形式的变量运算或指针寻址,switch-case语句的case标签必须是在编译时就能确定的整型常量
    switch (GET_CAN_REPLY_FRAME(CAN_Rx_Buffer->Header.ExtId))
    {
        //通信类型0：获取设备ID
        case (0x0):
        {
            RS05_Manage_Object->Target_Motor_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFFFF00) >> 8;
            //8字节数据包为MCU的ID，暂时不做处理
        }
        break;
        //通信类型2：电机反馈数据
        case (0x2):
        {
            RS05_Manage_Object->Target_Motor_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFF00) >> 8;
            RS05_Manage_Object->Fault_Information = (CAN_Rx_Buffer->Header.ExtId & 0x3F0000) >> 16;
            RS05_Manage_Object->Gridlock_Overload_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x100000) >> 20;
            RS05_Manage_Object->Magnetic_Coding_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x80000) >> 19;
            RS05_Manage_Object->Overtemperature = (CAN_Rx_Buffer->Header.ExtId & 0x40000) >> 18;
            RS05_Manage_Object->Three_Phase_Overcurrent_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x20000) >> 17;
            RS05_Manage_Object->Undervoltage_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x10000) >> 16;
            RS05_Manage_Object->Mode_Status = (CAN_Rx_Buffer->Header.ExtId & 0xC00000) >> 22;

            //8字节数据包解析
            RS05_Manage_Object->Current_Angle = (CAN_Rx_Buffer->Rx_Data[0] << 8 ) | CAN_Rx_Buffer->Rx_Data[1];
            RS05_Manage_Object->Current_Omega = (CAN_Rx_Buffer->Rx_Data[2] << 8 ) | CAN_Rx_Buffer->Rx_Data[3];
            RS05_Manage_Object->Current_Torque = (CAN_Rx_Buffer->Rx_Data[4] << 8 ) | CAN_Rx_Buffer->Rx_Data[5];
            RS05_Manage_Object->Current_Temperature = (float)((int16_t)((CAN_Rx_Buffer->Rx_Data[6] << 8 ) |
                                                      CAN_Rx_Buffer->Rx_Data[7])) / 10.0f;
        }
        break;
        //通信类型17：单参数读取
        case (0x11):
        {
            RS05_Manage_Object->Master_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFF00) >> 8;
            RS05_Manage_Object->Master_ID_Is_Read = (CAN_Rx_Buffer->Header.ExtId & 0xFF0000) >> 16;

            RS05_Manage_Object->Parameter_Data = CAN_Rx_Buffer->Rx_Data[7] << 24 | CAN_Rx_Buffer->Rx_Data[6] << 16 |
                                                 CAN_Rx_Buffer->Rx_Data[5] << 8 | CAN_Rx_Buffer->Rx_Data[4];
        }
        break;
        //通信类型21：故障反馈帧
        case (0x15):
        {
            RS05_Manage_Object->Target_Motor_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFF00) >> 8;

            RS05_Manage_Object->Fault_Value = CAN_Rx_Buffer->Rx_Data[3] << 24 | CAN_Rx_Buffer->Rx_Data[2] << 16 |
                                              CAN_Rx_Buffer->Rx_Data[1] << 8 | CAN_Rx_Buffer->Rx_Data[0];
            RS05_Manage_Object->A_Phase_Overcurrent_Sampling = CAN_Rx_Buffer->Rx_Data[2] & 0x1;
            RS05_Manage_Object->Motor_Stall_Overload_Algorithm_Protection = CAN_Rx_Buffer->Rx_Data[1] & 0x40;
            RS05_Manage_Object->Position_Initialization_Fault = CAN_Rx_Buffer->Rx_Data[1] & 0x2;
            RS05_Manage_Object->Hardware_Identification_Fault = CAN_Rx_Buffer->Rx_Data[1] & 0x1;
            RS05_Manage_Object->Encoder_Uncalibrated = CAN_Rx_Buffer->Rx_Data[0] & 0x80;
            RS05_Manage_Object->C_Phase_Overcurrent_Sampling = CAN_Rx_Buffer->Rx_Data[0] & 0x20;
            RS05_Manage_Object->B_Phase_Overcurrent_Sampling = CAN_Rx_Buffer->Rx_Data[0] & 0x10;
            RS05_Manage_Object->Overvoltage_Fault = CAN_Rx_Buffer->Rx_Data[0] & 0x08;
            RS05_Manage_Object->Undervoltage_Fault = CAN_Rx_Buffer->Rx_Data[0] & 0x04;
            RS05_Manage_Object->Driver_Chip_Fault = CAN_Rx_Buffer->Rx_Data[0] & 0x02;
            RS05_Manage_Object->Motor_Overtemperature_Fault = CAN_Rx_Buffer->Rx_Data[0] & 0x01;
            RS05_Manage_Object->Warning_Value = CAN_Rx_Buffer->Rx_Data[7] << 24 | CAN_Rx_Buffer->Rx_Data[6] << 16 |
                                                CAN_Rx_Buffer->Rx_Data[5] << 8 | CAN_Rx_Buffer->Rx_Data[4];
            RS05_Manage_Object->Motor_Overtemperature_Warning = CAN_Rx_Buffer->Rx_Data[4] & 0x1;
        }
        break;
        //通信类型24：电机主动报告帧数
        case (0x18):
        {
            RS05_Manage_Object->Target_Motor_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFF00) >> 8;
            RS05_Manage_Object->Fault_Information = (CAN_Rx_Buffer->Header.ExtId & 0x3F0000) >> 16;
            RS05_Manage_Object->Gridlock_Overload_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x100000) >> 20;
            RS05_Manage_Object->Magnetic_Coding_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x80000) >> 19;
            RS05_Manage_Object->Overtemperature = (CAN_Rx_Buffer->Header.ExtId & 0x40000) >> 18;
            RS05_Manage_Object->Three_Phase_Overcurrent_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x20000) >> 17;
            RS05_Manage_Object->Undervoltage_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x10000) >> 16;
            RS05_Manage_Object->Mode_Status = (CAN_Rx_Buffer->Header.ExtId & 0xC00000) >> 22;

            //8字节数据包
            RS05_Manage_Object->Current_Angle = (CAN_Rx_Buffer->Rx_Data[0] << 8) | CAN_Rx_Buffer->Rx_Data[1];
            RS05_Manage_Object->Current_Omega = (CAN_Rx_Buffer->Rx_Data[2] << 8) | CAN_Rx_Buffer->Rx_Data[3];
            RS05_Manage_Object->Current_Torque = (CAN_Rx_Buffer->Rx_Data[4] << 8) | CAN_Rx_Buffer->Rx_Data[5];
            RS05_Manage_Object->Current_Temperature = (float)((int16_t)((CAN_Rx_Buffer->Rx_Data[6] << 8 ) |
                                                      CAN_Rx_Buffer->Rx_Data[7])) / 10.0f;
        }
        break;
        //通信类型26：版本号读取帧，通信类型2和通信类型26的反馈帧的29位标识符的bit28~24一样，而且通信类型26不常用
        //如果要使用通信类型26，记得注释通信类型2
        // case (0x2):
        // {
        //     RS05_Manage_Object->Target_Motor_CAN_ID = (CAN_Rx_Buffer->Header.ExtId & 0xFF00) >> 8;
        //     RS05_Manage_Object->Fault_Information = (CAN_Rx_Buffer->Header.ExtId & 0x3F0000) >> 16;
        //     RS05_Manage_Object->Gridlock_Overload_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x100000) >> 20;
        //     RS05_Manage_Object->Magnetic_Coding_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x80000) >> 19;
        //     RS05_Manage_Object->Overtemperature = (CAN_Rx_Buffer->Header.ExtId & 0x40000) >> 18;
        //     RS05_Manage_Object->Three_Phase_Overcurrent_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x20000) >> 17;
        //     RS05_Manage_Object->Undervoltage_Fault = (CAN_Rx_Buffer->Header.ExtId & 0x10000) >> 16;
        //     RS05_Manage_Object->Mode_Status = (CAN_Rx_Buffer->Header.ExtId & 0xC00000) >> 22;
        //
        //     //8字节数据包，字节3~6为电机版本号，暂时不做处理
        // }
        // break;
    }

    //else if链在最坏情况下的执行效率是 O(N)，对于高频的CAN中断来说不够理想

}

// //控制模式———————————————————————————————————————————————————————————————————————————————————————————————————————————————
// //运控模式,上电之后的默认模式
// void Operation_Control_Mode(CAN_HandleTypeDef *hcan, float Target_Torque, float Target_Angle,
//                             float Target_Omega, float Send_Kp, float Send_Kd,
//                             Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送电机使能运行（通信类型 3）
//     uint32_t ID = Motor_Enable_Run_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送运控模式电机控制指令（通信类型 1）
//     uint16_t Torque = float_to_uint16 (Target_Torque, -5.5f, 5.5f, 16);
//     ID = Operation_Control_ID(Torque, RS05_Manage_Object);
//     CAN_Tx_Data_Operation_Control_Packet(Target_Angle, Target_Omega, Send_Kp, Send_Kd, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //参数依次为计算出的期望转矩，微分系数/阻尼，目标角速度，实际速度，比例系数，目标位置/角度，实际位置/角度，前馈转矩
//     //t_ref = Kd * (v_vset - v_actual) + Kp * (p_set - p_actual) + t_ff;电机内部闭环控制计算公式
// }
//
// //电流模式
// void Current_Mode(CAN_HandleTypeDef *hcan, float iq_ref, Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送单参数写入指令（通信类型 18）,将 runmode（运行模式）参数设置为 3
//     uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7005, 3, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机使能运行帧（通信类型 3）
//     ID = Motor_Enable_Run_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //再次发送单参数写入指令（通信类型 18）,设置 iq_ref 参数为你想要设定的目标电流指令值
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Iq_ref = {.f = iq_ref};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7006, Iq_ref.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
// }
//
// //速度模式
// void Velocity_Mode(CAN_HandleTypeDef *hcan, float limit_cur, float acc_rad, float spd_ref,
//                    Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送电机模式参数写入指令（通信类型 18），将 runmode（运行模式）参数设置为 2
//     uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7005, 2, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机使能运行帧（通信类型 3）
//     ID = Motor_Enable_Run_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 limit_cur（电流限制）参数设置为默认的最大电流指令16
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Limit_cur = {.f = limit_cur};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7018, Limit_cur.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 acc_rad（加速度）参数设置为默认的加速度指令20
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Acc_rad = {.f = acc_rad};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7022, Acc_rad.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 spd_ref（目标速度）参数设置
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Spd_ref = {.f = spd_ref};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x700A, Spd_ref.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
// }
//
// //位置模式（CSP）
// void Location_Mode_CSP(CAN_HandleTypeDef *hcan, float limit_spd, float loc_ref,
//                        Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送电机模式参数写入指令（通信类型 18），将 runmode（运行模式）参数设置为 5
//     uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7005, 5, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机使能运行帧（通信类型 3）
//     ID = Motor_Enable_Run_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 limit_spd（速度限制）参数设置为默认的最大速度指令
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Limit_spd = {.f = limit_spd};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7017, Limit_spd.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 loc_ref（目标位置）参数设置为默认的位置指令,手册没说
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Loc_ref_CSP = {.f = loc_ref};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7016, Loc_ref_CSP.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
// }
//
// //位置模式（PP）
// void Location_Mode_PP(CAN_HandleTypeDef *hcan, float vel_max, float acc_set, float loc_ref,
//                       Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送电机模式参数写入指令（通信类型 18），将 runmode（运行模式）参数设置为 1
//     uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7005, 1, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机使能运行帧（通信类型 3）
//     ID = Motor_Enable_Run_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 vel_max 参数设置为默认的最大速度指令10
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Vel_max = {.f = vel_max};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7024, Vel_max.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 acc_set 参数设置为默认的加速度指令10
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Acc_set = {.f = acc_set};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7025, Acc_set.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
//
//     //发送电机模式参数写入指令（通信类型 18），将 loc_ref 参数设置为默认的位置指令
//     ID = Single_Parameter_Write_ID(RS05_Manage_Object);
//     Float_Uint_TypeDef Loc_ref_PP = {.f = loc_ref};
//     CAN_Tx_Data_Single_Parameter_Write_Packet(0x7016, Loc_ref_PP.u, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
// }

// //停止驱动模式
// void  Stop_Running_Mode(CAN_HandleTypeDef *hcan, Motor_Manage_Object* RS05_Manage_Object)
// {
//     //发送电机停止帧（通信类型4）
//     uint32_t ID = Motor_Stop_Running_ID(RS05_Manage_Object);
//     CAN_Tx_Data_Motor_Stop_Running_Packet(0, RS05_Manage_Object);
//     CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
// }


//发送RS05电机使能运行（通信类型 3）
void RS05_Motor_Enable(CAN_HandleTypeDef *hcan, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Motor_Enable_Run_ID(RS05_Manage_Object);
    CAN_Tx_Data_Motor_Enable_Run_Packet(RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送运控模式电机控制指令（通信类型 1）
void Operation_Control_Mode(CAN_HandleTypeDef *hcan, float Target_Torque, float Target_Angle,
                            float Target_Omega, float Send_Kp, float Send_Kd,
                            Motor_Manage_Object* RS05_Manage_Object)
{
    uint16_t Torque = float_to_uint16 (Target_Torque, -5.5f, 5.5f, 16);
    uint32_t ID = Operation_Control_ID(Torque, RS05_Manage_Object);
    CAN_Tx_Data_Operation_Control_Packet(Target_Angle, Target_Omega, Send_Kp, Send_Kd, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);

    //参数依次为计算出的期望转矩，微分系数/阻尼，目标角速度，实际速度，比例系数，目标位置/角度，实际位置/角度，前馈转矩
    //t_ref = Kd * (v_vset - v_actual) + Kp * (p_set - p_actual) + t_ff;电机内部闭环控制计算公式
}

//发送单参数写入指令（通信类型 18）,设置runmode（运行模式）参数;0: 运控模式;1: 位置模式（PP）;2: 速度模式;3: 电流模式;5:位置模式（CSP）
void Set_Mode(CAN_HandleTypeDef *hcan, uint8_t runmode, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7005, runmode, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置电流模式 Iq 指令
void Set_iq_ref(CAN_HandleTypeDef *hcan, float iq_ref, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Iq_ref = {.f = iq_ref};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7006, Iq_ref.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置速度位置模式电流限制为默认的最大电流指令16
void Set_limit_cur(CAN_HandleTypeDef *hcan, float limit_cur, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Limit_cur = {.f = limit_cur};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7018, Limit_cur.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置速度模式加速度acc_rad为默认的加速度指令20
void Set_acc_rad(CAN_HandleTypeDef *hcan, float acc_rad, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Acc_rad = {.f = acc_rad};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7022, Acc_rad.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置转速模式转速指令
void Set_spd_ref(CAN_HandleTypeDef *hcan, float spd_ref, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Spd_ref = {.f = spd_ref};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x700A, Spd_ref.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置位置模式（CSP）速度限制
void Set_limit_spd(CAN_HandleTypeDef *hcan, float limit_spd, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Limit_spd = {.f = limit_spd};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7017, Limit_spd.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置位置模式角度指令
void Set_loc_ref(CAN_HandleTypeDef *hcan, float loc_ref, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Loc_ref = {.f = loc_ref};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7016, Loc_ref.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置位置模式（PP）速度为默认值10rad/s
void Set_vel_max(CAN_HandleTypeDef *hcan, float vel_max, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Vel_max = {.f = vel_max};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7024, Vel_max.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送单参数写入指令（通信类型 18）,设置
void Set_acc_set(CAN_HandleTypeDef *hcan, float acc_set, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Single_Parameter_Write_ID(RS05_Manage_Object);
    Float_Uint_TypeDef Acc_set = {.f = acc_set};
    CAN_Tx_Data_Single_Parameter_Write_Packet(0x7025, Acc_set.u, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}

//发送电机停止帧（通信类型4）
void RS05_Motor_Stop(CAN_HandleTypeDef *hcan, Motor_Manage_Object* RS05_Manage_Object)
{
    uint32_t ID = Motor_Stop_Running_ID(RS05_Manage_Object);
    CAN_Tx_Data_Motor_Stop_Running_Packet(0, RS05_Manage_Object);
    CAN_Send_Data(hcan, ID, RS05_Manage_Object->CAN_Tx_Data, 8);
}
