//
// AE483GroundStation
// David Hanley
//
// planner.c
// This file contains all functions for planning. The most basic example
// would be to hold position at some desired (x,y,z,yaw).
//

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/

//system
//#include "stdafx.h"
#include <math.h>
//planner
#include "planner.h"

// //////////////////////////////
// FIXME
//

// Parameters
float katt = 1;
float batt = 1;
float krep = 1;
float brep = 1;
float kdes = 1;
float bdes = 1;

// Drone radius
float r = 0.2;

// Goal position
float qgoal[3] = { 2.0, 0.0, -1.0 };

// Obstacle radius
float s = 0.2;

//
// //////////////////////////////

// Variables to keep track of time
float prev_time = 0;
float delta_t;
int firsttime = 1;

/*------------- End Includes -------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*----------------------------- Functions ------------------------------*/
/*----------------------------------------------------------------------*/

/*-- Potential Field --*/

// MAKE YOUR EDITS BELOW HERE

void PotentialField(State6_f state_param, State6_f ObstCurrent, PosYaw_f *dPosDes, float time, int PotFlag)
{
    float q[3];         // desired position
    float p[3];         // obstacle position
    float d;            // distance to obstacle
    float dgrad[3];     // gradient of distance to obstacle
    float fgrad[3];     // gradient of potential function
    float v[3];         // you can use this to store a vector (e.g., q - qgoal, or q - p)
    float vnorm;        // you can use this to store the norm of a vector (e.g., ||q - qgoal||, or ||q - p||)
    int i;              // you can use this for a counter (e.g., in a "for" loop)

    if (PotFlag == 1)   // DO COLLISION AVOIDANCE
    {
        // Get the desired position
        if (firsttime)
        {
            // Initialize the desired position at the quadrotor's current position
            q[0] = state_param.Pos.x;
            q[1] = state_param.Pos.y;
            q[2] = state_param.Pos.z;
            firsttime = 0;
        }
        else
        {
            q[0] = dPosDes->Pos.x;
            q[1] = dPosDes->Pos.y;
            q[2] = dPosDes->Pos.z;
        }

        // Get the obstacle position
        p[0] = ObstCurrent.Pos.x;
        p[1] = ObstCurrent.Pos.y;
        p[2] = ObstCurrent.Pos.z;

        // Initialize gradient
        fgrad[0] = 0;
        fgrad[1] = 0;
        fgrad[2] = 0;

        // Add attractive part of gradient (FIXME)
        //
        //  After some intermediate calculations, your code should
        //  do something like:
        //
        //      fgrad[0] += ... ;
        //      fgrad[1] += ... ;
        //      fgrad[2] += ... ;
        //

        // Add repulsive part of gradient (FIXME)
        //
        //  After some intermediate calculations, your code should
        //  do something like:
        //
        //      fgrad[0] += ... ;
        //      fgrad[1] += ... ;
        //      fgrad[2] += ... ;
        //

        // Take a step (FIXME)
        //
        //  After some intermediate calculations, your code should
        //  do something like:
        //
        //      q[0] += ... ;
        //      q[1] += ... ;
        //      q[2] += ... ;
        //

        // Set the desired position
        dPosDes->Pos.x = q[0];
		dPosDes->Pos.y = q[1];
		dPosDes->Pos.z = q[2];
        dPosDes->Tz = 0;
    }
    else                // LAND ON GROUND
    {

        dPosDes->Pos.x = 0;
		dPosDes->Pos.y = 0;
		dPosDes->Pos.z = -1.0;
		dPosDes->Tz = 0;
    }
}
/* End Potential Field */



/*--------------- Quat2Euler/Euler2Quat ---------------*/
void Planner_Quat2Euler_f(Quat_f *quat_param, Euler_f *euler_param)
{
	//variables
	float sqw = quat_param->qw*quat_param->qw;
	float sqx = quat_param->qx*quat_param->qx;
	float sqy = quat_param->qy*quat_param->qy;
	float sqz = quat_param->qz*quat_param->qz;
	float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor

										//singularity tests
	float test = quat_param->qx*quat_param->qy + quat_param->qz*quat_param->qw;
	if (test > 0.499*unit) { // singularity at north pole
		euler_param->ty = 2 * atan2f(quat_param->qx, quat_param->qw);
		euler_param->tz = PI_f / 2;
		euler_param->tx = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		euler_param->ty = -2 * atan2f(quat_param->qx, quat_param->qw);
		euler_param->tz = -PI_f / 2;
		euler_param->tx = 0;
		return;
	}

	//no singularity
	euler_param->ty = atan2f(2 * quat_param->qy*quat_param->qw - 2 * quat_param->qx*quat_param->qz, sqx - sqy - sqz + sqw);
	euler_param->tz = asinf(2 * test / unit);
	euler_param->tx = atan2f(2 * quat_param->qx*quat_param->qw - 2 * quat_param->qy*quat_param->qz, -sqx + sqy - sqz + sqw);
}
/*------------- End Quat2Euler/Euler2Quat -------------*/

/*----- MakeState -----*/
void Planner_MakeState6f_PosQuat(State6_f *state_param, Pos_f *pos_param, Quat_f *quat_param)
{
	state_param->Pos.x = pos_param->x;
	state_param->Pos.y = pos_param->y;
	state_param->Pos.z = pos_param->z;
	Planner_Quat2Euler_f(quat_param, &(state_param->Ori));
}
/*--- End MakeState ---*/



/*----------------------------------------------------------------------*/
/*--------------------------- End Functions ----------------------------*/
/*----------------------------------------------------------------------*/
