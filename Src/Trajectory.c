/*

 * Trajectory.c
 *
 *  Created on: 23.08.2019
 *      Author: Kuba*/

#include "Trajectory.h"
#include "InverseKinematics.h"

int trajectory_number=0;
float x,y,z,roll,pitch,yaw;
float pos=0;
float trajectory_speed=0.1;
float theta=0;
float vtheta=0.01;
float radius=200;
int part=0;
int counter=0;

void trajectory_generator(int num)
{
	yaw = 0;
	pitch = M_PI_2;
	roll = 0;
	x = 500;
	theta += vtheta;
	pos += trajectory_speed;
	float xd;
	if (theta >= 2 * M_PI)
	{
		theta = 0;
	}
	switch (num)
	{
	case 0:

		y = radius * cosf(theta);
		z = 500 + radius * sinf(theta);
		break;
	case 1:
		radius = 150;
		counter++;
		y = radius * sinf(theta);
		switch (part)
		{
		case 0:
			z = 400 + radius * cosf(theta);
			break;
		case 1:
			xd = theta + M_PI;
			if (xd > M_PI)
				xd += (-(2 * M_PI));
			z = 700 + radius * cosf(xd);
			break;
		}
		if (z >= 545 && z <= 555 && counter >= 200)
		{
			if (part == 0)
			{
				part = 1;
			}
			else if (part == 1)
			{
				part = 0;
			}
			else
			{
				part = 0;
			}
			counter = 0;
		}
		break;
	case 2:
		switch (part)
		{
		case 0:
			y = -250 + pos;
			z = 800;
			if (y >= 250)
			{
				pos = 0;
				part++;
			}
			break;
		case 1:
			y = 250;
			z = 800 - pos;
			if (z<=300)
			{
				pos = 0;
				part++;
			}
			break;
		case 2:
			y = 250 - pos;
			z = 300;
			if (y <= -250)
			{
				pos = 0;
				part++;
			}
			break;
		case 3:
			y = -250;
			z = 300 + pos;
			if (z >= 800)
			{
				pos = 0;
				part++;
			}
			break;
		}
		if (part == 4)
		{
			part = 0;
		}
		break;
		case 3:
			switch(part)
			{
			case 0:
				y = -300 + pos;
				z = 600 - pos;
				if (y >= 0)
				{
					pos = 0;
					part++;
				}
				break;
			case 1:
				y = 0 + pos;
				z = 300 + pos;
				if (z >= 600)
				{
					pos = 0;
					part++;
				}
				break;
			case 2:
				y = 250 - pos;
				z = 600;
				if (y <= -300)
				{
					pos = 0;
					part++;
				}
				break;
			}
			if(part>2)
			{
				part = 0;
			}
			break;
	}

	kinematics_in.f[0] = x;
	kinematics_in.f[1] = y;
	kinematics_in.f[2] = z;
	kinematics_in.f[3] = yaw;
	kinematics_in.f[4] = pitch;
	kinematics_in.f[5] = roll;

}
