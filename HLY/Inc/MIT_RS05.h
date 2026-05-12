//
// Created by 34285 on 2026/5/9.
//

#ifndef RS05_MIT_RS05_H
#define RS05_MIT_RS05_H

#include "stm32f4xx_hal.h"
#include "RS05_common.h"
#include "drv_math.h"

#define PI 3.14159265358979323846f

void MIT_CAN_Tx_Data_Enable_Motor_Operation_Packet(Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Stop_Motor_Operation_Packet(Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_MIT_Dynamic_Parameters_Packet(float Target_Angle, float Target_Omega,
                                                   float Send_Kp, float Send_Kd, float Target_Torque,
                                                   Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Set_Zero_Position_Packet(Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Clear_Errors_And_Read_Fault_Status_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Set_Operation_Mode_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Modify_Motor_CAN_ID_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Change_Communication_Protocol_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Modify_Host_CAN_ID_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Position_Mode_Control_Packet(float Target_Position, float Target_Speed,
                                                  Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Velocity_Mode_Control_Packet(float Target_Speed, float Current_Limit,
                                                  Motor_Manage_Object *RS05_Manage_Object );
void MIT_CAN_Tx_Data_Motor_Data_Save_Packet(Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Motor_Active_Report_Packet(uint8_t F_CMD, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Motor_Parameter_Read_Packet(uint16_t index, Motor_Manage_Object *RS05_Manage_Object);
void MIT_CAN_Tx_Data_Motor_Parameter_Write_Packet(uint16_t index, uint32_t parameter_data,
                                                  Motor_Manage_Object *RS05_Manage_Object);

void MIT_CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t Object_Para, uint32_t Filter_ID, uint32_t Mask_ID);

uint16_t MIT_Position_Mode_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object);
uint16_t MIT_Velocity_Mode_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object);
uint16_t MIT_Motor_Parameter_Read_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object);
uint16_t MIT_Motor_Parameter_Write_Calculate_ID(Motor_Manage_Object *RS05_Manage_Object);

uint8_t MIT_CAN_Send_Data(CAN_HandleTypeDef* hcan, uint16_t CAN_ID, uint8_t *Data, uint16_t Length);
void MIT_CAN_RS05_Call_Back(Struct_CAN_Rx_Buffer* CAN_Rx_Buffer, Motor_Manage_Object* MIT_RS05_Manage_Object);

void MIT_Motion_Control_Mode(CAN_HandleTypeDef* hcan, float Target_Angle, float Target_Omega,
                             float Send_Kp, float Send_Kd, float Target_Torque,
                             Motor_Manage_Object* RS05_Manage_Object);
void MIT_Velocity_Mode(CAN_HandleTypeDef* hcan, float Target_Speed, float Current_Limit,
                       Motor_Manage_Object* RS05_Manage_Object);
void MIT_Position_Mode_CSP(CAN_HandleTypeDef* hcan, float Target_Position, float Target_Speed,
                           Motor_Manage_Object* RS05_Manage_Object);
void MIT_Stop_Mode(CAN_HandleTypeDef* hcan, Motor_Manage_Object* RS05_Manage_Object);

#endif //RS05_MIT_RS05_H
