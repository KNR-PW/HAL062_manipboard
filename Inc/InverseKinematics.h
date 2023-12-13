/*
 * InverseKinematics.h
 *
 *  Created on: 11.05.2019
 *      Author: Adam
 */

#ifndef INC_INVERSEKINEMATICS_H_
#define INC_INVERSEKINEMATICS_H_

#include "stm32f4xx_hal.h"
#include "math.h"
#include "can.h"
#include "ForwardKinematics.h"

/*kolejno x, y, z w mm*/

#define DOF2_3Distance 550.0
#define DOF3_5Distance 510.0
#define GripperLenght 100

extern enum MMode ManipulatorMode;
extern volatile uint8_t newmode;

extern uint8_t GripperState;

struct Position
{
	float x;
	float y;
	float z;
};

struct Orientation
{
	float yaw;
	float pitch;
	float roll;
};

enum MMode
{
	DISABLED=0,			//wylaczone sterowniki
	HOLD,				//trzymanie pozycji
	ANGLES,				//podawanie kolejnych katow dof
	VELOCITY,			//praca na predkosci zlaczowa
	XYZ_GLOBAL,
	VEL_GLOB,
	VEL_TOOL,
	DELTA_TOOL,
	TRAJECTORY
};

enum Disconnect
{
	NO,
	DOF6,
	DOF456
};


union F2I
{
	float f[6];
	uint32_t i[6];
};

extern enum Disconnect DisconnectDoF;
extern union F2I angle, speed, meas_angle, read_value, kinematics_in;


void StateFoo();
void StateFooLF();
void InverseKinematicsInit();
void CalculateLowerAngles();
void CalculateUpperAngles();
void CheckSolutions();
void CalculateInverseKinematics();
void TransformToolFull(float VectorIn[6], float VectorOut[6]);

#endif /* INC_INVERSEKINEMATICS_H_ */
