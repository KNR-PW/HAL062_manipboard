/*
 * ForwardKinematics.c
 *
 *  Created on: 27.05.2019
 *      Author: Adam
 */

#ifndef INC_FORWARDKINEMATICS_C_
#define INC_FORWARDKINEMATICS_C_

#include "main.h"
#include "InverseKinematics.h"

void DH_Matrix (arm_matrix_instance_f32 *T, float alpha, float theta, float a, float d);


void CalculateForwardKinematics(float Joints[6],float Position[6][3], uint8_t mode, arm_matrix_instance_f32  *DHMatrix);
#endif /* INC_FORWARDKINEMATICS_C_ */
