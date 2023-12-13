/*
 * InverseKinematics.c
 *
 *  Created on: 11.05.2019
 *      Author: Adam
 */

#include "InverseKinematics.h"

#define DOF1MAX 	M_PI * 2.0/3.0
#define DOF1MIN 	-M_PI * 2.0/3.0
#define DOF2MAX 	M_PI_2
#define DOF2MIN 	-M_PI/3
#define DOF3MAX 	M_PI/3.0
#define DOF3MIN 	-M_PI/3.0
#define DOF4MAX		M_PI
#define DOF4MIN		-M_PI
#define DOF5MAX		M_PI * 13.0/18.0
#define DOF5MIN		-M_PI * 4.0/9.0
#define DOF6MAX		M_PI
#define DOF6MIN		-M_PI

#define TOLERANCE 0.01

#define VELOCITY_DELTA_T 0.01

struct Position WristPosition;
struct Position TCPPosition;
struct Position Velocity;

struct Orientation  TCPOrientation, Omega;

enum MMode ManipulatorMode;

enum Disconnect DisconnectDoF;

union F2I angle, speed, meas_angle, read_value, kinematics_in;

uint8_t GripperState;		//tryb chwytaka 0 - STOP, 1 - OTWORZ, 2 - ZAMKNIJ
float JointVelocity;
volatile uint8_t newmode;		//flaga oznaczajaca nowy tryb

const float DOF_MAX[] =
{ DOF1MAX, DOF2MAX, DOF3MAX, DOF4MAX, DOF5MAX, DOF6MAX };
const float DOF_MIN[] =
{ DOF1MIN, DOF2MIN, DOF3MIN, DOF4MIN, DOF5MIN, DOF6MIN };


arm_matrix_instance_f32 DH_T06;
arm_matrix_instance_f32 DH_TCPtoWCP;
arm_matrix_instance_f32 DH_WCP;
arm_matrix_instance_f32 DH_T03;

volatile float UpperAngles[2][3];
volatile float LowerAngles[4][3];
volatile float Solutions[8][6];

float DH_T06_f32[16];
float DH_TCPtoWCP_f32[4];
float DH_WCP_f32[4];
float DH_T03_f32[16];
uint8_t SolutionsCorrect[8];
uint8_t choosedSolution;
float CalculateDistance(uint8_t index, int DOFs);
uint8_t CheckLimits(uint8_t index, uint8_t DOFs);
void CalculateUpperAngles();
void CheckSolutions();
void NormalizeSolutions();
int ChooseSolution();
void WriteSolution(int index);
void TransformToolWrist(float VectorIn[3], float VectorOut[3]);
void CalculatePositionIncrement();
void InvertDHMatrix(arm_matrix_instance_f32 *In, arm_matrix_instance_f32 *Out);
void StateFooLF() {

	/*oldmode*/
	/*If manipulatorMode!=oldmode*/
/*	{
		HOLD
		oldMode=Manipulatormode

		Dla VEL_GLOBAL. VEL_TOOL. Pobierz katy i forward_kinematics -> TCP/WCP.

	}*/
	switch (ManipulatorMode) {
	case DISABLED:
		/*Wyslij disable*/
		break;
	case HOLD:
		/*Nic*/
		break;
	case ANGLES:
		/*Wyslij katy*/
		break;
	case VELOCITY:
		/*Wyslij predkosc*/
		break;
	case XYZ_GLOBAL:
		/*podstaw kinematics in jako TCP.Position/Orienation*/
		/*Policz inversekinematics*/
		if (DisconnectDoF == DOF6)
		{

		}
		else if (DisconnectDoF == DOF456)
		{

		}
		break;
	case VEL_GLOB:
		/*Przelicz predkosci na delty pozycji*/
		if (DisconnectDoF == DOF6)
		{

		}
		else if (DisconnectDoF == DOF456)
		{

		}
		break;
	case VEL_TOOL:
		//transformTool();
		if (DisconnectDoF == DOF6) {

		} else if (DisconnectDoF == DOF456) {

		}
		break;
	case DELTA_TOOL:
		//transformTool();
		if (DisconnectDoF == DOF6) {

		} else if (DisconnectDoF == DOF456) {

		}
		break;
	}
	if (ManipulatorMode > VELOCITY) {
		CalculateInverseKinematics();
	}
}
void CalculatePositionIncrement()
{

	if (ManipulatorMode==VEL_GLOB)
	{
		if (DisconnectDoF==DOF456)
		{
			WristPosition.x=WristPosition.x+Velocity.x*VELOCITY_DELTA_T;
			WristPosition.y=WristPosition.y+Velocity.y*VELOCITY_DELTA_T;
			WristPosition.z=WristPosition.z+Velocity.z*VELOCITY_DELTA_T;

		}
		else
		{
			TCPPosition.x=TCPPosition.x+Velocity.x*VELOCITY_DELTA_T;
			TCPPosition.y=TCPPosition.y+Velocity.y*VELOCITY_DELTA_T;
			TCPPosition.z=TCPPosition.z+Velocity.z*VELOCITY_DELTA_T;
			TCPOrientation.yaw=TCPOrientation.yaw+Omega.yaw*VELOCITY_DELTA_T;
			TCPOrientation.pitch=TCPOrientation.pitch+Omega.pitch*VELOCITY_DELTA_T;
			TCPOrientation.roll=TCPOrientation.roll+Omega.roll*VELOCITY_DELTA_T;
		}
	}
	if (ManipulatorMode==VEL_TOOL)
	{
		if(DisconnectDoF==DOF456)
		{
			float tempVector[]={Velocity.x,Velocity.y,Velocity.z};
			float VectorOut[3];
			TransformToolWrist(tempVector,VectorOut);
			WristPosition.x=WristPosition.x+VectorOut[0]*VELOCITY_DELTA_T;
			WristPosition.y=WristPosition.y+VectorOut[1]*VELOCITY_DELTA_T;
			WristPosition.z=WristPosition.z+VectorOut[2]*VELOCITY_DELTA_T;
		}
		else
		{
			float tempVector[]={Velocity.x,Velocity.y,Velocity.z,Omega.yaw,Omega.pitch,Omega.roll};
			float TransformedVector[6];
			TransformToolFull(tempVector,TransformedVector);
			TCPPosition.x=TCPPosition.x+TransformedVector[0]*VELOCITY_DELTA_T;
			TCPPosition.y=TCPPosition.y+TransformedVector[1]*VELOCITY_DELTA_T;
			TCPPosition.z=TCPPosition.z+TransformedVector[2]*VELOCITY_DELTA_T;
			TCPOrientation.yaw=TCPOrientation.yaw+TransformedVector[3]*VELOCITY_DELTA_T;
			TCPOrientation.pitch=TCPOrientation.pitch+TransformedVector[4]*VELOCITY_DELTA_T;
			TCPOrientation.roll=TCPOrientation.roll+TransformedVector[5]*VELOCITY_DELTA_T;


		}
	}

}
/*atan 2 ma kolejno y,x. Zwraca wartosci od (-pi;pi>*/
void InverseKinematicsInit()
{
	/*	WristPosition.x=400.0;
	 WristPosition.y=.0;
	 WristPosition.z=700.0;*/
	TCPPosition.x = 400.0;
	TCPPosition.y = 000.0;
	TCPPosition.z = 750.0;
	/*	WristPosition.x=300.0;
	 WristPosition.y=100.0;
	 WristPosition.z=300.0;*/
	TCPOrientation.yaw = 0;
	TCPOrientation.pitch = M_PI_2;
	TCPOrientation.roll = 0;
	arm_mat_init_f32(&DH_T03, 4, 4, DH_T03_f32);
	arm_mat_init_f32(&DH_T06, 4, 4, DH_T06_f32);
	arm_mat_init_f32(&DH_TCPtoWCP, 4, 1, DH_TCPtoWCP_f32);
	arm_mat_init_f32(&DH_WCP, 4, 1, DH_WCP_f32);

	DH_TCPtoWCP_f32[0] = 0.0;
	DH_TCPtoWCP_f32[1] = 0.0;
	DH_TCPtoWCP_f32[2] = -GripperLenght;
	DH_TCPtoWCP_f32[3] = 1.0;
}
void CalculateInverseKinematics()
{
	arm_matrix_instance_f32 DHMatrixTemp;
	float DHMatrixTemp_f32[16];
	arm_mat_init_f32(&DHMatrixTemp,4,4,DHMatrixTemp_f32);

	/*Step 1. Load input*/
	if (ManipulatorMode == XYZ_GLOBAL || ManipulatorMode == TRAJECTORY)
	{
		if (DisconnectDoF == DOF456)
		{
			WristPosition.x = kinematics_in.f[0];
			WristPosition.y = kinematics_in.f[1];
			WristPosition.z = kinematics_in.f[2];
		}
		else
		{
			TCPPosition.x = kinematics_in.f[0];
			TCPPosition.y = kinematics_in.f[1];
			TCPPosition.z = kinematics_in.f[2];
			TCPOrientation.yaw = kinematics_in.f[3];
			TCPOrientation.pitch = kinematics_in.f[4];
			TCPOrientation.roll = kinematics_in.f[5];
		}
	}
	else if (ManipulatorMode==VEL_GLOB || ManipulatorMode==VEL_TOOL)
	{
		if (DisconnectDoF == DOF456)
		{
			Velocity.x=kinematics_in.f[0];
			Velocity.y=kinematics_in.f[1];
			Velocity.z=kinematics_in.f[2];
			Omega.yaw=0.0;
			Omega.pitch=0.0;
			Omega.roll=0.0;
		}
		else
		{
			Velocity.x=kinematics_in.f[0];
			Velocity.y=kinematics_in.f[1];
			Velocity.z=kinematics_in.f[2];
			Omega.yaw=kinematics_in.f[3];
			Omega.pitch=kinematics_in.f[4];
			Omega.roll=kinematics_in.f[5];
		}
		CalculatePositionIncrement(&DHMatrixTemp);
	}
	else if (ManipulatorMode==DELTA_TOOL)
	{
		float TransforedIncrements[6];
		TransformToolFull(kinematics_in.f,TransforedIncrements);
		TCPPosition.x = TCPPosition.x+TransforedIncrements[0];
		TCPPosition.y = TCPPosition.y+TransforedIncrements[1];
		TCPPosition.z = TCPPosition.z+TransforedIncrements[2];
		TCPOrientation.yaw = TCPOrientation.yaw + TransforedIncrements[3];
		TCPOrientation.pitch = TCPOrientation.pitch + TransforedIncrements[4];
		TCPOrientation.roll = TCPOrientation.roll + TransforedIncrements[5];
	}

	/*Step 2. Create transformation matrix if working with more than 3 DOFs*/
	if (DisconnectDoF != DOF456)
	{

		DH_T06_f32[0] = cosf(TCPOrientation.yaw) * cosf(TCPOrientation.pitch);
		DH_T06_f32[1] = cosf(TCPOrientation.yaw) * sinf(TCPOrientation.pitch)
				* sinf(TCPOrientation.roll)
				- sinf(TCPOrientation.yaw) * cosf(TCPOrientation.roll);
		DH_T06_f32[2] = cosf(TCPOrientation.yaw) * sinf(TCPOrientation.pitch)
				* cosf(TCPOrientation.roll)
				+ sinf(TCPOrientation.yaw) * sinf(TCPOrientation.roll);
		DH_T06_f32[3] = TCPPosition.x;
		DH_T06_f32[4] = sinf(TCPOrientation.yaw) * cosf(TCPOrientation.pitch);
		DH_T06_f32[5] = sinf(TCPOrientation.yaw) * sinf(TCPOrientation.pitch)
				* sinf(TCPOrientation.roll)
				+ cosf(TCPOrientation.yaw) * cosf(TCPOrientation.roll);
		DH_T06_f32[6] = sinf(TCPOrientation.yaw) * sinf(TCPOrientation.pitch)
				* cosf(TCPOrientation.roll)
				- cosf(TCPOrientation.yaw) * sinf(TCPOrientation.roll);
		DH_T06_f32[7] = TCPPosition.y;
		DH_T06_f32[8] = -sinf(TCPOrientation.pitch);
		DH_T06_f32[9] = -cosf(TCPOrientation.pitch) * sinf(TCPOrientation.roll);
		DH_T06_f32[10] = cosf(TCPOrientation.pitch) * cosf(TCPOrientation.roll);
		DH_T06_f32[11] = TCPPosition.z;
		DH_T06_f32[12] = 0.0;
		DH_T06_f32[13] = 0.0;
		DH_T06_f32[14] = 0.0;
		DH_T06_f32[15] = 1.0;
		arm_mat_mult_f32(&DH_T06, &DH_TCPtoWCP, &DH_WCP);
		WristPosition.x = DH_WCP.pData[0];
		WristPosition.y = DH_WCP.pData[1];
		WristPosition.z = DH_WCP.pData[2];
	}
	/*Step 3. Calculate inverse kinematics. If working with DOF 1-3 only calculate Lower Angles*/
	if (DisconnectDoF != DOF456)
	{
		CalculateLowerAngles();
		CalculateUpperAngles();
	}
	else
	{
		CalculateLowerAngles();
	}
	NormalizeSolutions();
	CheckSolutions();
	int x = ChooseSolution(choosedSolution);
	choosedSolution = x;
	if(x<9)
	{
		WriteSolution(x);
	}
	return;
}
void CalculateLowerAngles()
{
	float thetaDOF1fwd, thetaDOF1bwd, temp1, temp2, thetaDOF2fwdTop,
			thetaDOF2bwdTop, thetaDOF3Top, thetaDOF2fwdBottom,
			thetaDOF2bwdBottom, thetaDOF3Bottom;
	if (fabs(WristPosition.x) < 0.001 && fabs(WristPosition.y) < 0.001)
	{
		thetaDOF1fwd = angle.f[0];
		thetaDOF1bwd = angle.f[0];
	}
	else
	{
		temp1 = atan2f(WristPosition.y, WristPosition.x);

		if (temp1 > 0.0)
			temp2 = temp1 - M_PI;
		else
			temp2 = temp1 + M_PI;
		if (WristPosition.x > 0.0)
		{
			thetaDOF1fwd = temp1;
			thetaDOF1bwd = temp2;
		}
		else
		{
			thetaDOF1fwd = temp2;
			thetaDOF1bwd = temp1;
		}
	}
	float x2y2 = powf(WristPosition.x, 2.0) + powf(WristPosition.y, 2.0);
	volatile float c2 = x2y2 + powf(WristPosition.z, 2.0);
	volatile float c = sqrtf(c2);
	volatile float alfa = acosf(
			((pow(DOF3_5Distance, 2.0) - pow(DOF2_3Distance, 2.0) - c2))
					/ (-2.0 * DOF2_3Distance * c));
	volatile float gamma = acosf(
			(c2 - pow(DOF2_3Distance, 2.0) - pow(DOF3_5Distance, 2.0))
					/ (-2.0 * DOF2_3Distance * DOF3_5Distance));
	volatile float delta_forward, delta_backward;

	if (WristPosition.x > 0.0)
	{
		delta_forward = atan2(WristPosition.z, sqrtf(x2y2));
		delta_backward = atan2(WristPosition.z, -sqrtf(x2y2));

	}
	else
	{
		delta_forward = atan2(WristPosition.z, -sqrtf(x2y2));
		delta_backward = atan2(WristPosition.z, sqrtf(x2y2));
	}
	thetaDOF2fwdTop = M_PI / 2 - (alfa + delta_forward);
	thetaDOF2bwdTop = M_PI / 2 - (alfa + delta_backward);
	thetaDOF3Top = M_PI / 2 - gamma;

	thetaDOF2fwdBottom = M_PI / 2 - (delta_forward - alfa);
	thetaDOF2bwdBottom = M_PI / 2 - (delta_backward - alfa);
	thetaDOF3Bottom = gamma - (3. / 2.) * M_PI;

	Solutions[0][0] = thetaDOF1fwd;
	Solutions[2][0] = thetaDOF1fwd;
	Solutions[4][0] = thetaDOF1bwd;
	Solutions[6][0] = thetaDOF1bwd;

	Solutions[0][1] = thetaDOF2fwdTop;
	Solutions[2][1] = thetaDOF2fwdBottom;
	Solutions[4][1] = thetaDOF2bwdTop;
	Solutions[6][1] = thetaDOF2bwdBottom;

	Solutions[0][2] = thetaDOF3Top;
	Solutions[2][2] = thetaDOF3Bottom;
	Solutions[4][2] = thetaDOF3Top;
	Solutions[6][2] = thetaDOF3Bottom;

	for (int j = 0; j <= 2; j++)
	{
		for (int i = 0; i <= 3; i++)
		{
			Solutions[2 * i + 1][j] = Solutions[2 * i][j];
		}
	}
}
void NormalizeSolutions()
{
	for (int i =0;i<8;i++)
	{
		for (int j=0;j<6;j++)
		{
			if (Solutions[i][j]>M_PI)
			{
				Solutions[i][j]=Solutions[i][j]-2*M_PI;
			}
			if (Solutions[i][j]<-M_PI)
			{
				Solutions[i][j]=Solutions[i][j]+2*M_PI;
			}
		}
	}
}
void CalculateUpperAngles()
{

	arm_matrix_instance_f32 temp, temp1, temp2, R36;
	float temp_f32[16], temp1_f32[16], temp2_f32[16], R36_f32[9];


	for (int i = 0; i < 4; i++)
	{
		arm_mat_init_f32(&temp, 4, 4, temp_f32);
		arm_mat_init_f32(&temp1, 4, 4, temp1_f32);
		arm_mat_init_f32(&temp2, 4, 4, temp2_f32);
		arm_mat_init_f32(&R36, 3, 3, R36_f32);
		DH_Matrix(&temp1, -M_PI_2, Solutions[2 * i][0], 0, 0);

		DH_Matrix(&temp, 0, Solutions[2 * i][1] - M_PI_2, DOF2_3Distance, 0);
		arm_mat_mult_f32(&temp1, &temp, &temp2);

		DH_Matrix(&temp, -M_PI_2, Solutions[2 * i][2], 0, 0);
		arm_mat_mult_f32(&temp2, &temp, &DH_T03);

		/*zamiana na macierz 3x3*/
		arm_mat_init_f32(&temp, 3, 3, temp_f32);
		arm_mat_init_f32(&temp1, 3, 3, temp1_f32);

		/*Kopiowanie macierzy rotacji R03*/
		temp.pData[0] = DH_T03.pData[0];
		temp.pData[1] = DH_T03.pData[1];
		temp.pData[2] = DH_T03.pData[2];
		temp.pData[3] = DH_T03.pData[4];
		temp.pData[4] = DH_T03.pData[5];
		temp.pData[5] = DH_T03.pData[6];
		temp.pData[6] = DH_T03.pData[8];
		temp.pData[7] = DH_T03.pData[9];
		temp.pData[8] = DH_T03.pData[10];

		/*Macierz odwrotna rotacji R30*/
		arm_mat_trans_f32(&temp, &temp1);

		/*Kopiowanie macierzy rotacji R06*/
		temp.pData[0] = DH_T06.pData[0];
		temp.pData[1] = DH_T06.pData[1];
		temp.pData[2] = DH_T06.pData[2];
		temp.pData[3] = DH_T06.pData[4];
		temp.pData[4] = DH_T06.pData[5];
		temp.pData[5] = DH_T06.pData[6];
		temp.pData[6] = DH_T06.pData[8];
		temp.pData[7] = DH_T06.pData[9];
		temp.pData[8] = DH_T06.pData[10];

		arm_mat_mult_f32(&temp1, &temp, &R36);

		Solutions[2 * i][4] = acosf(R36.pData[8]);
		Solutions[2 * i + 1][4] = -Solutions[2 * i][4];

		/*Gimbal lock*/
		if (fabs(R36.pData[8]) < 0.001)
		{
			Solutions[2 * i][3] = angle.f[3];
			Solutions[2 * i + 1][3] = angle.f[3];
			Solutions[2 * i][5] = asin(R36.pData[1]) - Solutions[2 * i][3];
			Solutions[2 * i + 1][5] = asin(R36.pData[1])
					- Solutions[2 * i + 1][3];
		}
		else
		{
			float tempsin1 = sinf(Solutions[2 * i][4]);
			float tempsin2 = sinf(Solutions[2 * i + 1][4]);

			Solutions[2 * i][3] = -atan2f((R36.pData[5] / tempsin1),
					(-R36.pData[2] / tempsin1));
			Solutions[2 * i + 1][3] = -atan2f((R36.pData[5] / tempsin2),
					(-R36.pData[2] / tempsin2));

			Solutions[2 * i][5] = atan2f((-R36.pData[7] / tempsin1),
					(R36.pData[6] / tempsin1));
			Solutions[2 * i + 1][5] = atan2f((-R36.pData[7] / tempsin2),
					(R36.pData[6] / tempsin2));
		}
	}
}

uint8_t CheckLimits(uint8_t index, uint8_t DOFs)
{
	uint8_t result=1;
	for (int j = 0; j < DOFs; j++)
	{
		if (Solutions[index][j] > (DOF_MAX[j] + TOLERANCE)
				|| Solutions[index][j] < (DOF_MIN[j] - TOLERANCE))
		{
			result =0;
		}
	}
	return result;
}
float CalculateDistance(uint8_t index, int DOFs)
{
	float sum = 0.0;
	for (int i = 0; i < DOFs; i++)
	{
		sum += powf((angle.f[i] - Solutions[index][i]), 2.0);
	}
	return sum;
}


void CheckSolutions()
{
	/*Check Solutions' limit*/
	uint8_t temp;
	switch (DisconnectDoF)
	{
	case DOF456:
		temp=3;
		break;
	case DOF6:
		temp=5;
		break;
	default:
		temp=6;
		break;

	}
	for (int i=0;i<8;i++)
	{
		/*Reset first*/
		SolutionsCorrect[i]=0;
		SolutionsCorrect[i]=CheckLimits(i,temp);
	}
	return;
}
void TransformToolFull(float VectorIn[6], float VectorOut[6])
{

	float temp[6][3], DHMat_f32[16], DHMatInv_f32[16], DHTool_f32[16], DHOutput_f32[16];
	float yaw = VectorIn[3], pitch=VectorIn[4], roll = VectorIn[5];


	arm_matrix_instance_f32 DHMat, DHTool,DHMatInv, DHOutput;
	arm_mat_init_f32(&DHMat,4,4,DHMat_f32);
	arm_mat_init_f32(&DHMatInv,4,4,DHMatInv_f32);
	arm_mat_init_f32(&DHTool,4,4,DHTool_f32);
	arm_mat_init_f32(&DHOutput,4,4,DHOutput_f32);


	/*Create DH Matrix for desired transforamtion*/
	DHTool_f32[0] = cosf(yaw) * cosf(pitch);
	DHTool_f32[1] = cosf(yaw) * sinf(pitch) * sinf(roll)
			- sinf(yaw) * cosf(roll);
	DHTool_f32[2] = cosf(yaw) * sinf(pitch) * cosf(roll)
			+ sinf(yaw) * sinf(roll);
	DHTool_f32[3] = VectorIn[0];
	DHTool_f32[4] = sinf(yaw) * cosf(pitch);
	DHTool_f32[5] = sinf(yaw) * sinf(pitch) * sinf(roll)
			+ cosf(yaw) * cosf(roll);
	DHTool_f32[6] = sinf(yaw) * sinf(pitch) * cosf(roll)
			- cosf(yaw) * sinf(roll);
	DHTool_f32[7] = VectorIn[1];
	DHTool_f32[8] = -sinf(pitch);
	DHTool_f32[9] = -cosf(pitch) * sinf(roll);
	DHTool_f32[10] = cosf(pitch) * cosf(roll);
	DHTool_f32[11] = VectorIn[2];
	DHTool_f32[12] = 0.0;
	DHTool_f32[13] = 0.0;
	DHTool_f32[14] = 0.0;
	DHTool_f32[15] = 1.0;

	/*Get DH Matrix for current position and invert it*/
	CalculateForwardKinematics(meas_angle.f,temp,0,&DHMat);
	InvertDHMatrix(&DHMat,&DHMatInv);

	/*Mutiply 2 matrcies*/
	arm_mat_mult_f32(&DHMatInv,&DHTool,&DHOutput);

	VectorOut[0]=DHOutput.pData[3];
	VectorOut[1]=DHOutput.pData[7];
	VectorOut[2]=DHOutput.pData[11];

	/*Pitch First. Patrz: DHTool_f32[8]*/
	VectorOut[4]=-asinf(DHOutput.pData[8]);

	/*Then roll.  Patrz: DHTool_f32[9]*/
	VectorOut[5]=-asinf(DHOutput.pData[9]/cosf(VectorOut[4]));

	/*Finally roll*. Patrz DHTool_f32[0]*/
	VectorOut[3]=acosf(DHOutput.pData[0]/cosf(VectorOut[4]));
	return;
}
void InvertDHMatrix(arm_matrix_instance_f32 *In, arm_matrix_instance_f32 *Out)
{

	/*https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Use_of_Denavit_and_Hartenberg_matrices*/
	arm_matrix_instance_f32 RotMat, RotMatT, OutPosition, InPosition;
	float RotMat_f32[9], RotMatT_f32[9], OutPosition_f32[3], InPosition_f32[3];
	arm_mat_init_f32(&RotMat,3,3,RotMat_f32);
	arm_mat_init_f32(&RotMatT,3,3,RotMatT_f32);
	arm_mat_init_f32(&OutPosition,3,1,OutPosition_f32);
	arm_mat_init_f32(&InPosition,3,1,InPosition_f32);

	/*Copy Rotation Matrix from input*/
	RotMat.pData[0]=In->pData[0];
	RotMat.pData[1]=In->pData[1];
	RotMat.pData[2]=In->pData[2];
	RotMat.pData[3]=In->pData[4];
	RotMat.pData[4]=In->pData[5];
	RotMat.pData[5]=In->pData[6];
	RotMat.pData[6]=In->pData[8];
	RotMat.pData[7]=In->pData[9];
	RotMat.pData[8]=In->pData[10];

	/*Invert Rotation Matrix <==> Transpose*/

	arm_mat_trans_f32(&RotMat,&RotMatT);

	/*Calculate T-part of Inverted DH-Matrix*/


	InPosition.pData[0]=In->pData[3];
	InPosition.pData[1]=In->pData[7];
	InPosition.pData[2]=In->pData[11];
	arm_mat_mult_f32(&RotMatT,&InPosition,&OutPosition);

	/*Copy everything*/

	Out->pData[0]=RotMatT.pData[0];
	Out->pData[1]=RotMatT.pData[1];
	Out->pData[2]=RotMatT.pData[2];
	Out->pData[3]=-OutPosition.pData[0];
	Out->pData[4]=RotMatT.pData[3];
	Out->pData[5]=RotMatT.pData[4];
	Out->pData[6]=RotMatT.pData[5];
	Out->pData[7]=-OutPosition.pData[1];
	Out->pData[8]=RotMatT.pData[6];
	Out->pData[9]=RotMatT.pData[7];
	Out->pData[10]=RotMatT.pData[8];
	Out->pData[11]=-OutPosition.pData[2];
	Out->pData[12]=0.0;
	Out->pData[13]=0.0;
	Out->pData[14]=0.0;
	Out->pData[15]=1.0;

}
void TransformToolWrist(float VectorIn[3], float VectorOut[3])
{
	VectorOut[2]=VectorIn[2];
	VectorOut[0]=VectorIn[0]*cosf(meas_angle.f[0])-VectorIn[1]*sinf(meas_angle.f[0]);
	VectorOut[1]=VectorIn[0]*sinf(meas_angle.f[0])+VectorIn[1]*cosf(meas_angle.f[0]);
}

int ChooseSolution(int last)
{
	int temp;
	float x;
	switch (DisconnectDoF)
	{
		case DOF456:
			temp=3;
			break;
		case DOF6:
			temp=5;
			break;
		default:
			temp=6;
			break;
	}
	int index = 10;

	if(last<9 && SolutionsCorrect[last]==1)
	{
		index=last;
		return index;
	}
	float sum = INFINITY;
	for (int i=0;i<8;i++)
	{
		if(SolutionsCorrect[i]==1)
		{
			x = CalculateDistance(i,temp);
			if(x < sum)
			{
				index = i;
				sum = x;
			}
		}
	}
	return index;
}

void WriteSolution(int index)
{
	int temp;
	switch (DisconnectDoF)
	{
		case DOF456:
			temp=3;
			break;
		case DOF6:
			temp=5;
			break;
		default:
			temp=6;
			break;
	}
	for(int i=0;i<temp; i++)
	{
		angle.f[i] = Solutions[index][i];
	}
}

