//
// Created by 34285 on 2026/4/28.
//

#ifndef RS05_DRV_RS05_H
#define RS05_DRV_RS05_H

#include "stm32f4xx_hal.h"
#include "RS05_common.h"

#define PI 3.14159265358979323846f

//RS05电机的29位ID包
//Bit28~bit24通信类型
#define CAN_ID_COMMUNICATION_TYPE(x) (((x) & 0x1F) << 24)
//bit23~8数据区域2
#define CAN_ID_DATA_AREA2(x) (((x) & 0xFFFF) << 8)
//bit7~0目标地址
#define CAN_ID_DESTINATION_ADDRESS(x) ((x) & 0xFF)

//获得电机反馈帧bit28~24和bit7~0的值
#define GET_CAN_REPLY_FRAME(x) (((x) & 0x1F000000) >> 24)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


void CAN_Tx_Data_Get_Device_ID_Packet(Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Operation_Control_Packet(float Target_Angle, float Target_Omega, float Send_Kp, float Send_Kd,
                                          Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Enable_Run_Packet(Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Stop_Running_Packet(uint8_t mode, Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Set_Motor_Mechanical_Zero_Packet(Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Single_Parameter_Read_Packet(uint16_t index, Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Single_Parameter_Write_Packet(uint16_t index, uint32_t parameter_data,
                                               Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Data_Save_Packet(Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Baud_Rate_Modification_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Actively_Report_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Motor_Protocol_Modification_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void CAN_Tx_Data_Version_Number_Read_Packet(Motor_Manage_Object *RS05_Manage_Object);


void CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID);

uint32_t Get_Device_ID_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Operation_Control_ID(uint16_t Target_Torque, Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Enable_Run_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Stop_Running_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Set_Motor_Mechanical_Zero_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Set_Motor_CAN_ID_ID(uint8_t Preset_CAN_ID, Motor_Manage_Object *RS05_Manage_Object);
uint32_t Single_Parameter_Read_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Single_Parameter_Write_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Data_Save_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Baud_Rate_Modification_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Actively_Report_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Motor_Protocol_Modification_ID(Motor_Manage_Object *RS05_Manage_Object);
uint32_t Version_Number_Read_ID(Motor_Manage_Object *RS05_Manage_Object);

uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint32_t CAN_ID, uint8_t *Data, uint16_t Length);

void CAN_RS05_Call_Back(Struct_CAN_Rx_Buffer* CAN_Rx_Buffer, Motor_Manage_Object* RS05_Manage_Object);

void Operation_Control_Mode(CAN_HandleTypeDef *hcan, float Target_Torque, float Target_Angle,
                            float Target_Omega, float Send_Kp, float Send_Kd,
                            Motor_Manage_Object* RS05_Manage_Object);
void Current_Mode(CAN_HandleTypeDef *hcan, float iq_ref, Motor_Manage_Object* RS05_Manage_Object);
void Velocity_Mode(CAN_HandleTypeDef *hcan, float limit_cur, float acc_rad, float spd_ref,
                   Motor_Manage_Object* RS05_Manage_Object);
void Location_Mode_CSP(CAN_HandleTypeDef *hcan, float limit_spd, float loc_ref,
                       Motor_Manage_Object* RS05_Manage_Object);
void Location_Mode_PP(CAN_HandleTypeDef *hcan, float vel_max, float acc_set, float loc_ref,
                      Motor_Manage_Object* RS05_Manage_Object);
void  Stop_Running_Mode(CAN_HandleTypeDef *hcan, Motor_Manage_Object* RS05_Manage_Object);

#endif //RS05_DRV_RS05_H
