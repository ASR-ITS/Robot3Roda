/*
 * motor.c
 *
 *  Created on: Sep 22, 2021
 *      Author: ismarintan
 */

#include "motor.h"
#include "can.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

CAN_TxHeaderTypeDef Tx_Msg;
CAN_RxHeaderTypeDef Rx_Msg;

uint8_t CAN_TxData[8];
uint8_t CAN_RxData[8];

uint32_t CAN_TxMailbox = 0;

int motorSpeedSP[3];



void Motor_Init(Motor_TypeDef *pMotor) {

	Tx_Msg.DLC = 8;
	Tx_Msg.StdId = 0x600 + pMotor->MotorID;
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;

	//Enable Control
	CAN_TxData[0] = 0x2B;
	CAN_TxData[1] = 0x40;
	CAN_TxData[2] = 0x60;
	CAN_TxData[3] = 0x00;

	CAN_TxData[4] = 0x2F;
	CAN_TxData[5] = 0x00;
	CAN_TxData[6] = 0x00;
	CAN_TxData[7] = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, CAN_TxData, &CAN_TxMailbox);
	HAL_Delay(5);

}

void MotorKontrolHandler(Motor_TypeDef *pMotor) {

	Tx_Msg.DLC = 8;
	Tx_Msg.StdId = 0x600 + pMotor->MotorID;
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;

	// set speed
	CAN_TxData[0] = 0x23;
	CAN_TxData[1] = 0x02;
	CAN_TxData[2] = 0x20;
	CAN_TxData[3] = 0x01;

	CAN_TxData[4] = (uint8_t) (pMotor->TargetSpeed & 0xFF);
	CAN_TxData[5] = (uint8_t) ((pMotor->TargetSpeed >> 8) & 0xFF);
	CAN_TxData[6] = (uint8_t) ((pMotor->TargetSpeed >> 16) & 0xFF);
	CAN_TxData[7] = (uint8_t) ((pMotor->TargetSpeed >> 24) & 0xFF);

	HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, (uint8_t*) CAN_TxData, &CAN_TxMailbox);

	HAL_Delay(50);

	// set speed
	CAN_TxData[0] = 0x23;
	CAN_TxData[1] = 0x01;
	CAN_TxData[2] = 0x20;
	CAN_TxData[3] = 0x01;

	CAN_TxData[4] = (uint8_t) (pMotor->TargetPosisi & 0xFF);
	CAN_TxData[5] = (uint8_t) ((pMotor->TargetPosisi >> 8) & 0xFF);
	CAN_TxData[6] = (uint8_t) ((pMotor->TargetPosisi >> 16) & 0xFF);
	CAN_TxData[7] = (uint8_t) ((pMotor->TargetPosisi >> 24) & 0xFF);

	HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, (uint8_t*) CAN_TxData, &CAN_TxMailbox);
	HAL_Delay(50);


}

void MotorReadVoltage(Motor_TypeDef *pMotor)
{
	Tx_Msg.DLC = 8;
	Tx_Msg.StdId = 0x600 + pMotor->MotorID;
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;

	CAN_TxData[0] = 0x40;
	CAN_TxData[1] = 0x0D;
	CAN_TxData[2] = 0x21;
	CAN_TxData[3] = 0x02;

	CAN_TxData[4] = 0;
	CAN_TxData[5] = 0;
	CAN_TxData[6] = 0;
	CAN_TxData[7] = 0;

	HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, (uint8_t*) CAN_TxData, &CAN_TxMailbox);


}


void motorLocalSpeed(int speedX,int speedY, int speedZ)
{

	motorSpeedSP[0] = -sinf((30+180)*0.0174533)*speedX + cosf((30+180)*0.0174533)*speedY + speedZ;
	motorSpeedSP[1] = -(-sinf((150+180)*0.0174533)*speedX + cosf((150+180)*0.0174533)*speedY + speedZ);
	motorSpeedSP[2] = -(-sinf((270+180)*0.0174533)*speedX + cosf((270+180)*0.0174533)*speedY + speedZ);

}



void MotorRxDataHandler(Motor_TypeDef *pMotor,CAN_RxHeaderTypeDef *pMsg,uint8_t CAN_Data[8])
{
	if(pMsg->StdId == pMotor->MotorID + 0x580)
	{

		if (CAN_Data[0]==0x4b && CAN_Data[1] == 0x0D && CAN_Data[2]==0x21 && CAN_Data[3]==0x02)
		{
			int16_t batSens;

			memcpy(&batSens, CAN_Data + 4, 2);

			pMotor->VoltRAW = (float)batSens*0.1;
			pMotor->Voltage = pMotor->Voltage*0.85 + pMotor->VoltRAW*0.15;

//
		}
	}
}
