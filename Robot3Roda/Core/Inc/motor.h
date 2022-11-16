/*
 * motor.h
 *
 *  Created on: Sep 22, 2021
 *      Author: ismarintan
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"


typedef enum{
	Speed = 0,
	Posisi = 1,

}Motor_Mode;

typedef struct {

	uint32_t MotorID;
	int32_t TargetPosisi;
	int32_t TargetSpeed;
	int32_t Akselerasi;
	int32_t Posisi;
	Motor_Mode MotorMode;
	float Voltage;
	float VoltRAW;


}Motor_TypeDef;


extern int motorSpeedSP[3];

void Motor_Init(Motor_TypeDef *pMotor);
void MotorKontrolHandler(Motor_TypeDef *pMotor);
void MotorRxDataHandler(Motor_TypeDef *pMotor,CAN_RxHeaderTypeDef *pMsg,uint8_t CAN_Data[8]);
void motorLocalSpeed(int speedX,int speedY, int speedZ);
void MotorReadVoltage(Motor_TypeDef *pMotor);

#endif /* INC_MOTOR_H_ */
