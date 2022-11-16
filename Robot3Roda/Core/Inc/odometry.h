/*
 * odometry.h
 *
 *  Created on: Oct 8, 2021
 *      Author: ismar
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "main.h"

extern float encWheel_Vel[2];
extern float odomLocalSpeed[3];
extern float odomGlobalSpeed[3];
extern float odomGlobalPosition[3];


void OdomCalc(TIM_HandleTypeDef *wheelENC0,TIM_HandleTypeDef *wheelENC1, float heading);

#endif /* INC_ODOMETRY_H_ */
