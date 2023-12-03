/*

 * ForwardKinematics.c
 *
 *  Created on: 27.05.2019
 *      Author: Adam*/

#include "ForwardKinematics.h"

void CopyDHToPoisition(arm_matrix_instance_f32  *DH, float *Position);

void DH_Matrix (arm_matrix_instance_f32 *T, float alpha, float theta, float a, float d)
{
	T->pData[0]=cosf(theta);
	T->pData[1]=-sinf(theta)*cosf(alpha);
	T->pData[2]=sinf(theta)*sinf(alpha);
	T->pData[3]=a*cosf(theta);

	T->pData[4]=sinf(theta);
	T->pData[5]=cosf(theta)*cosf(alpha);
	T->pData[6]=-sinf(alpha)*cosf(theta);
	T->pData[7]=sinf(theta)*a;

	T->pData[8]=0.0;
	T->pData[9]=sinf(alpha);
	T->pData[10]=cosf(alpha);
	T->pData[11]=d;

	T->pData[12]=0.0;
	T->pData[13]=0.0;
	T->pData[14]=0.0;
	T->pData[15]=1.0;
}
void CopyDHToPoisition(arm_matrix_instance_f32  *DH, float Position[])
{
	Position[0]=DH->pData[3];
	Position[1]=DH->pData[7];
	Position[2]=DH->pData[11];
}
void CalculateForwardKinematics(float Joints[], float Position[][3],
		uint8_t mode, arm_matrix_instance_f32 *DHMatrix)
{
	/*mode=0 zwraca macierz T05
	 * mode-1 zwraca macierz T06
	 */
	arm_matrix_instance_f32 temp, temp1, temp2;
	float temp_f32[16], temp1_f32[16], temp2_f32[16];
	arm_mat_init_f32(&temp, 4, 4, temp_f32);
	arm_mat_init_f32(&temp1, 4, 4, temp1_f32);
	arm_mat_init_f32(&temp2, 4, 4, temp2_f32);

	/*1 DOF*/
	DH_Matrix(&temp1, -M_PI / 2, Joints[0], 0, 0);
	CopyDHToPoisition(&temp1, Position[0]);

	/*2 DOF*/
	DH_Matrix(&temp, 0, Joints[1]-M_PI/2, DOF2_3Distance, 0);
	arm_mat_mult_f32(&temp1, &temp, &temp2);
	CopyDHToPoisition(&temp2, Position[1]);

	/*3 DOF*/
	DH_Matrix(&temp, -M_PI / 2, Joints[2], 0, 0);
	arm_mat_mult_f32(&temp2, &temp, &temp1);
	CopyDHToPoisition(&temp1, Position[2]);

	/*4 DOF*/
	DH_Matrix(&temp, M_PI / 2, Joints[3], 0, DOF3_5Distance);
	arm_mat_mult_f32(&temp1, &temp, &temp2);
	CopyDHToPoisition(&temp2, Position[3]);

	if (mode == 0)
	{

		/*5 DOF*/
		DH_Matrix(&temp, -M_PI / 2, Joints[4], 0, 0);
		arm_mat_mult_f32(&temp2, &temp, DHMatrix);
		CopyDHToPoisition(DHMatrix, Position[4]);

		/*6 DOF*/
		DH_Matrix(&temp, 0, Joints[5], 0, GripperLenght);
		arm_mat_mult_f32(DHMatrix, &temp, &temp2);
		CopyDHToPoisition(&temp2, Position[5]);

	}
	else
	{
		/*5 DOF*/
		DH_Matrix(&temp, -M_PI / 2, Joints[4], 0, 0);
		arm_mat_mult_f32(&temp2, &temp, &temp1);
		CopyDHToPoisition(&temp1, Position[4]);

		/*6 DOF*/
		DH_Matrix(&temp, 0, Joints[5], 0, GripperLenght);
		arm_mat_mult_f32(&temp1, &temp, DHMatrix);
		CopyDHToPoisition(DHMatrix, Position[5]);
	}

}

