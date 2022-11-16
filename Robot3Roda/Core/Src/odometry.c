/*
 * odometry.c
 *
 *  Created on: Oct 8, 2021
 *      Author: ismar
 */

#include "odometry.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "tim.h"
#include "math.h"

float encWheel_Vel[2];
float odomLocalSpeed[3];
float odomGlobalSpeed[3];
float odomGlobalPosition[3];

// diameter roda 6 cm
// ppr enc 360


/*
 * Matrix Inverse
 *
 * -0.707107	-0.707107
 *  0.707107	-0.707107
*/

void OdomCalc(TIM_HandleTypeDef *wheelENC0,TIM_HandleTypeDef *wheelENC1, float heading)
{

	short int encBuff[2];

	encBuff[0] = wheelENC0->Instance->CNT;
	encBuff[1] = wheelENC1->Instance->CNT;

	encWheel_Vel[0] = encBuff[0];
	encWheel_Vel[1] = encBuff[1];

	wheelENC0->Instance->CNT=0;
	wheelENC1->Instance->CNT=0;

	// 0.01667 = diameter / ppr * 4
	odomLocalSpeed[0] = ((encWheel_Vel[0]*-0.707107)+(encWheel_Vel[1]*-0.707107));
	odomLocalSpeed[1] = ((encWheel_Vel[0]*0.707107)+(encWheel_Vel[1]*-0.707107));

	odomGlobalSpeed[0] = cosf(heading*0.0174533)*odomLocalSpeed[0] - sinf(heading*0.0174533)*odomLocalSpeed[1];
	odomGlobalSpeed[1] = sinf(heading*0.0174533)*odomLocalSpeed[0] + cosf(heading*0.0174533)*odomLocalSpeed[1];

	odomGlobalPosition[0] += odomGlobalSpeed[0]*0.01667;
	odomGlobalPosition[1] += odomGlobalSpeed[1]*0.01667;
	odomGlobalPosition[2] = heading;


}
