/*
 * lab.h
 *
 *  Created on: Jan 26, 2015
 *      Author: hanley6
 */

#ifndef LAB_H_
#define LAB_H_

#include "main.h"
#include "sdk.h"
#include "uart.h"
#include "stdint.h"
#include "Utilites.h"

int takeoff2;

//Velocity Estimation
int initialize;
int velocityEstHolder;

//Low Pass Filter
float VelEstX;
float VelEstY;
float VelEstZ;
float previousXMeas;
float previousYMeas;
float previousZMeas;

//Integral of error
float errorcum[3];
int IntegralHolder;

struct imuSensor{
    float dT;	        // time stamp (elapsed time since startup in seconds)

    float dThetax;      // Roll Euler angle
    float dThetay;	// Pitch Euler angle
    float dThetaz;	// Yaw Euler angle

    float dVx;		// Velocity (m/s) at x-axis
    float dVy;		// Velocity (m/s) at y-axis
    float dVz;		// Velocity (m/s) at z-axis

    float dOmegax;	// Angular velocity (rad/s) about the body-x-axis
    float dOmegay;	// Angular velocity (rad/s) about the body-y-axis
    float dOmegaz;	// Angular velocity (rad/s) about the body-z-axis

    float dAx;		// Acceleration along body x-axis
    float dAy;		// Acceleration along body y-axis
    float dAz;		// Acceleration along body z-axis

    float dHx;		// Magnetic field component along body x-axis (NOTE. CONFIRM THIS IS TRUE BEFORE USING)
    float dHy;		// Magnetic field component along body y-axis (NOTE. CONFIRM THIS IS TRUE BEFORE USING)
    float dHz;		// Magnetic field component along body z-axis (NOTE. CONFIRM THIS IS TRUE BEFORE USING)
 

    float dPressure;	// Barometric pressure
};

extern struct imuSensor imusensor;


struct MOCAP{
    uint16_t iFrame;	// Frame Number
    float dT;		// Time Stamp

    float dX;		// x-position
    float dY;		// y-position
    float dZ;		// z-position
    float dThetax;	// Roll Euler angle
    float dThetay;	// Pitch Euler angle
    float dThetaz;	// Yaw Euler angle

    float dVx;		// Velocity (m/s) at x-axis
    float dVy;		// Velocity (m/s) at y-axis
    float dVz;		// Velocity (m/s) at z-axis
};

extern struct MOCAP mocap;
extern struct MOCAP mocapOld;

// Control Inputs
struct U {
	float u1;
	float u2;
	float u3;
	float u4;
};

extern struct U u;

// Planner Inputs
struct GoalPosYaw {
	float x;
	float y;
	float z;
	float yaw;
};

extern struct GoalPosYaw goalposyaw;

void lab(void);

#endif /* LAB_H_ */
