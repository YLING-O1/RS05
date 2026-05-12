//
// Created by 34285 on 2026/5/10.
//

#include "RS05_common.h"

//CAN处理接收到的对应结构体
Struct_CAN_Manage_Object CAN1_Manage_Object;
Struct_CAN_Manage_Object CAN2_Manage_Object;

//RS05电机信息结构体
Motor_Manage_Object RS05_Motor_Manage_Object;

//初始化CAN总线——————————————————————————————————————————————————————————————————————————————————————————————————————————
void CAN_Init(CAN_HandleTypeDef* hcan,CAN_Call_Back CallBack_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.CallBack_Function = CallBack_Function;
        CAN1_Manage_Object.Motor_Count = 0;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.CallBack_Function = CallBack_Function;
        CAN2_Manage_Object.Motor_Count = 0;
    }
}

//电机注册
void Motor_Register(Struct_CAN_Manage_Object* CAN_Manage_Object, Motor_Manage_Object* Motor_Object)
{
    if (CAN_Manage_Object->Motor_Count < MAX_MOTOR_COUNT)
    {
        CAN_Manage_Object->Motor_List[CAN_Manage_Object->Motor_Count] = Motor_Object;
        CAN_Manage_Object->Motor_Count++;
    }
}

//CAN接收FIFO0中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Struct_CAN_Rx_Buffer CAN_Rx_Buffer;

    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Buffer.Header, CAN_Rx_Buffer.Rx_Data);
        //判空保护
        if (CAN1_Manage_Object.CallBack_Function != NULL)
        {
            //遍历总线上注册的所有电机
            for (uint8_t i = 0; i < CAN1_Manage_Object.Motor_Count; i++)
            {
                CAN1_Manage_Object.CallBack_Function(&CAN_Rx_Buffer, CAN1_Manage_Object.Motor_List[i]);
            }
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Buffer.Header, CAN_Rx_Buffer.Rx_Data);
        if (CAN2_Manage_Object.CallBack_Function != NULL)
        {
            //遍历总线上注册的所有电机
            for (uint8_t i = 0; i < CAN2_Manage_Object.Motor_Count; i++)
            {
                CAN2_Manage_Object.CallBack_Function(&CAN_Rx_Buffer, CAN2_Manage_Object.Motor_List[i]);
            }
        }
    }
}

//CAN接收FIFO1中断回调函数
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Struct_CAN_Rx_Buffer CAN_Rx_Buffer;

    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_Rx_Buffer.Header, CAN_Rx_Buffer.Rx_Data);
        if (CAN1_Manage_Object.CallBack_Function != NULL)
        {
            //遍历总线上注册的所有电机
            for (uint8_t i = 0; i < CAN1_Manage_Object.Motor_Count; i++)
            {
                CAN1_Manage_Object.CallBack_Function(&CAN_Rx_Buffer, CAN1_Manage_Object.Motor_List[i]);
            }
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_Rx_Buffer.Header, CAN_Rx_Buffer.Rx_Data);
        if (CAN2_Manage_Object.CallBack_Function != NULL)
        {
            //遍历总线上注册的所有电机
            for (uint8_t i = 0; i < CAN2_Manage_Object.Motor_Count; i++)
            {
                CAN2_Manage_Object.CallBack_Function(&CAN_Rx_Buffer, CAN2_Manage_Object.Motor_List[i]);
            }
        }
    }
}

