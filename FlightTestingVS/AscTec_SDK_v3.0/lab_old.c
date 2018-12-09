/*
 * lab.c
 *
 *  Created on: Jan 26, 2015
 *      Author: hanley6
 */

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/
#include "lab.h"
#include "math.h"
/*------------- End Includes -------------*/

/*---------- Function Prototypes ---------*/
void lab3(void);
void convert_input_to_motor_commands(void);
void do_sensor_fusion(void);
/*-------- End Function Prototypes -------*/

/*--------------- Globals ----------------*/
struct imuSensor imusensor;
struct U u;
struct MOCAP mocap;
struct GoalPosYaw goalposyaw;

/////////// HUMMINGBIRD PARAMETERS /////////////
float mass = 0.715;
float kF = 7.46e-6;
float kM = 1.23e-7;
float l = 0.17;
float MAXPHI2 = 779.5638*779.5638;
float MINPHI2 = 112.705875*112.705875;
////////////////////////////////////////////

/////////// CONTROLLER PARAMETERS /////////////
// The gain matrix has size 4x12 but is represented in C as an array
// with 48 elements. The first 12 elements are in the first row of the
// matrix, the second 12 in the second row, and so forth.
//float K[48] = {   0.000000000000014 ,  1.875007700503404 ,  0.000000000000054 ,  0.000000000000000 , -0.000000000000001 ,  2.070418318602924 ,  0.000000000000002 ,  0.893576448635991 ,  0.000000000000047 ,  0.152828058872079 , -0.000000000000000 ,  0.000000000000000,
//				 -1.850925589056405 , -0.000000000000002 ,  0.000000000000061 ,  0.000000000000002 ,  2.037739961934616 , -0.000000000000002 , -0.880796758304062 , -0.000000000000001 ,  0.000000000000051 , -0.000000000000000 ,  0.149448159491756 ,  0.000000000000000,
//				 -0.000000000000018 , -0.000000000000015 ,  0.000000000000027 ,  1.649473343767278 ,  0.000000000000004 , -0.000000000000005 , -0.000000000000007 , -0.000000000000004 ,  0.000000000000004 ,  0.000000000000000 ,  0.000000000000000 ,  0.189557251102787,
//				  0.000000000000044 , -0.000000000000062 , -6.492988611013611 , -0.000000000000000 , -0.000000000000019 , -0.000000000000003 ,  0.000000000000015 , -0.000000000000011 , -3.050131746457585 , -0.000000000000000 , -0.000000000000000 , -0.000000000000000};

float K[48] = {1.01676031032402e-14,	1.33662498985831,	-8.94632153765127e-14,	8.99810275643853e-16,	-9.50500537961551e-15,	1.96375913607365,	1.03055572186138e-14,	0.736393665410587,	-9.46494516859666e-14,	0.150686199408661,	-1.59320477688817e-16,	2.53501140693094e-16,
               -1.31945838148381,	-4.84632938698589e-15,	7.57666561054556e-14,	3.36966530095013e-15,	1.93351517046756,	-1.29065404216143e-15,	-0.726006017482509,	6.59927114526977e-16,	4.97974932953041e-14,	-8.70124242068185e-17,	0.147400785014897,	2.17281608893389e-16,
               -2.19100114709845e-14,	-6.52827908061739e-15,	-1.89030143807745e-13,	1.64947334376730,	8.21865819792227e-15,	1.99304455219464e-15,	-1.14191809589057e-14,	-4.70270166379141e-15,	-1.25982580980028e-13,	1.54783813399836e-16,	1.70148843544250e-16,	0.189557251102789,
               4.31816403353325e-14,	9.34624296010056e-14,	-4.62311824144024,	1.39838607200238e-14,	-2.22552613985841e-14,	5.26724511837937e-14,	2.78716474282077e-14,	4.80205188601509e-14,	-2.57480960382173,	1.03232188436190e-15,	-4.74973560028025e-16,	1.45764654924504e-15};

//-6.5 whatever
////////////////////////////////////////////


// Other variables you'll need to work with
float cnt_u[4];
float o_desired[3];
float yaw_desired;
float g = 9.80665;	// Standard Gravity m/s^2
float xd[12];   // discrete state (i.e., state error)

// Other variables you can ignore for now
float Winv[4*4];
float omega_cmd2[4];
float omega_cmd[4];
float cmd[4];

/*------------- End Globals --------------*/

/*----------------------------------------------------------------------*/
/*---------------------------- End Preamble ----------------------------*/
/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/
/*------------------ Main Loop (called at 1 kHz) -----------------------*/
/*----------------------------------------------------------------------*/
void lab(void)
{
	// Desired Position
	o_desired[0] = goalposyaw.x;	// x/North (m)
	o_desired[1] = goalposyaw.y;	// y/East (m)
	o_desired[2] = goalposyaw.z;	// z/Down (m)
	yaw_desired = goalposyaw.yaw;	// yaw (rad)

	// Fuse mocap measurements with IMU measurements
	do_sensor_fusion();

	// Run controller to choose inputs (modify struct u)
	lab3();

	// Convert inputs to motor commands
	convert_input_to_motor_commands();
}
/*----------------------------------------------------------------------*/
/*---------------- End Main Loop (called at 1 kHz) ---------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Helpers -------------------------------*/
/*----------------------------------------------------------------------*/
/*---------- Velocity Estimator ----------*/
int LED1toggle = 0;
int LED1count = 0;
int numCMDs = 0.0;

void do_sensor_fusion() {

    // Only proceed if the mocap measurements have changed
	if ( (previousXMeas != mocap.dX) || (previousYMeas != mocap.dY) || (previousZMeas != mocap.dZ) ) {
		numCMDs = numCMDs + 1.0;
		LED1count++;
		if (0==(LED1count%10)) {
			if (LED1toggle == 0) {
				LED(1,OFF);
				LED1toggle = 1;
			} else {
				LED(1,ON);
				LED1toggle = 0;
			}
		}
 		// Velocity Estimation
		if (initialize == 0)
		{
			// Initialization
			mocap.dX = mocap.dX;
			mocap.dY = mocap.dY;
			mocap.dZ = mocap.dZ;
			mocap.dVx = 0.0;
			mocap.dVy = 0.0;
			mocap.dVz = 0.0;

			initialize = 1;
		}
		else
		{

			mocap.dVx = (mocap.dX - previousXMeas)/0.02;
			mocap.dVy = (mocap.dY - previousYMeas)/0.02;
			mocap.dVz = (mocap.dZ - previousZMeas)/0.02;

			// Set Position
			mocap.dX = mocap.dX;
			mocap.dY = mocap.dY;
			mocap.dZ = mocap.dZ;

			/*--- End Low Pass Filtering ---*/
		}
	}
	// Save Current MoCap Measurement as Previous Measurement
	previousXMeas = mocap.dX;
	previousYMeas = mocap.dY;
	previousZMeas = mocap.dZ;

}

/*-------- End Velocity Estimator --------*/



/*-------------- Controller --------------*/
int LED0timeCount = 0;
int LED0toggle = 0;

void lab3() {

    ////////////////////////
    // DO NOT MODIFY
    //
    // Turn LEDs on and off for diagnostics
	if (LED0timeCount == 250) {
		LED0timeCount = 0;
		if (LED0toggle == 0) {
			LED(0,OFF);
			LED0toggle = 1;
		} else {
			LED(0,ON);
			LED0toggle = 0;
		}
	}
	LED0timeCount++;
    //
    ////////////////////////

    // Compute: xd = x - xe
    //
    //  xd, a 12x1 matrix, is represented in C code as a float array of length 12
    //  If you order states as o, theta, v, w, then:
    //      xd[0] is o1 - o1e
    //      xd[1] is o2 - o2e
    //      ...
    //      xd[11] is w3 - w3e
    //
    //  measurement of o:
    //      mocap.dX, mocap.dY, mocap.dZ
    //  measurement of theta:
    //      mocap.dThetaz, imusensor.dThetay, imusensor.dThetax
    //  measurement of v:
    //      mocap.dVx, mocap.dVy, mocap.dVz
    //  measurement of w:
    //      imusensor.dOmegax, imusensor.dOmegay, imusensor.dOmegaz
    //
    // CHANGE!!!


    xd[0] = mocap.dX - goalposyaw.x;
    xd[1] = mocap.dY - goalposyaw.y;
    xd[2] = mocap.dZ - goalposyaw.z;
    xd[3] =  mocap.dThetaz - 0.0;
    xd[4] =  imusensor.dThetay - 0.0;
    xd[5] =  imusensor.dThetax - 0.0;
    xd[6] = mocap.dVx - 0.0;
    xd[7] = mocap.dVy - 0.0;
    xd[8] = mocap.dVz - 0.0;
    xd[9] = imusensor.dOmegax - 0.0;
    xd[10] = imusensor.dOmegay - 0.0;
    xd[11] = imusensor.dOmegaz - 0.0;

    //float C[4];
    // Compute: K*(x - xe)
    //
    //  To find C = A * B, where A is mxn and B is nxp, use:
    //      matrix_multiply(m, n, p, A, B, C);
    matrix_multiply(4, 12, 1, K, xd, cnt_u);

    //  In particular, you may wish to compute:
    //      cnt_u = K * xd
    //
    //  ("cnt_u" is the variable in which you need to put
    //   the input; for now, it is a convenient place to
    //   store the result of this intermediate calculation)
    //
    // CHANGE!!!

    float ue[4] = {0,0,0,mass*g};
    // Compute: ue - K*(x - xe)
    //
    //  You should put the result in cnt_u.
    //
    // CHANGE!!!
    cnt_u[0] = ue[0] - cnt_u[0];
    cnt_u[1] = ue[1] - cnt_u[1];
    cnt_u[2] = ue[2] - cnt_u[2];
    cnt_u[3] = ue[3] - cnt_u[3];

    ////////////////////////
    // DO NOT MODIFY
    //
	// Store result
	u.u1 = cnt_u[0];
	u.u2 = cnt_u[1];
	u.u3 = cnt_u[2];
	u.u4 = cnt_u[3];
    //
    ////////////////////////
}
/*------------ End Lab 3 ------------*/

/*------------ End Controller ------------*/

/*---------------- Command ---------------*/
void convert_input_to_motor_commands() {
	/////////////// Controller Settings ////////////
	WO_SDK.ctrl_mode=0x00;  //0x00: direct individual motor control (individual commands for motors 0...3)
				//0x01: direct motor control using standard output mapping: commands are interpreted as pitch, roll, yaw
				//      and thrust inputs; no attitude controller active
				//0x02: attitude and throttle control: commands are input for standard attitude controller
				//0x03: GPS waypoint control

	WO_SDK.ctrl_enabled=1;	//0: disable control by HL processor
				//1: enable control by HL processor
	////////////////////////////////////////////////

	//////// Translate commanded torques and thrust into rotor speed and commands ////////////
	// NOTE METHOD BELOW ASSUMES THAT CG IS IN THE SAME PLANE AS THE ROTORS
	float twolkF = 1.0/(2.0*l*kF);
	float fourkF = 1.0/(4.0*kF);
	float fourkM = 1.0/(4.0*kM);

	Winv[0] = 0;
	Winv[1] = twolkF;
	Winv[2] = -fourkM;
	Winv[3] = fourkF;
	Winv[1*4+0] = -twolkF;
	Winv[1*4+1] = 0;
	Winv[1*4+2] = fourkM;
	Winv[1*4+3] = fourkF;
	Winv[2*4+0] = 0;
	Winv[2*4+1] = -twolkF;
	Winv[2*4+2] = -fourkM;
	Winv[2*4+3] = fourkF;
	Winv[3*4+0] = twolkF;
	Winv[3*4+1] = 0;
	Winv[3*4+2] = fourkM;
	Winv[3*4+3] = fourkF;

	matrix_multiply(4,4,1,Winv,cnt_u,omega_cmd2);

	int i;
	for (i=0; i<4; i++) {
		if (omega_cmd2[i] > MAXPHI2) {
			omega_cmd2[i] = MAXPHI2;
		}
		else if (omega_cmd2[i] < MINPHI2) {
			omega_cmd2[i] = MINPHI2;
		}
		omega_cmd[i] = sqrt(omega_cmd2[i]);
		// Translate Desired Rotor Speed into Motor Commands
		// NOTE: THIS IS FOR THE PELICAN
		cmd[i] = 0.238432*omega_cmd[i] - 25.872642;	// Verify

		// Below is a safety measure. We want to make sure the motor
		// commands are never 0 so that the motors will always keep
		// spinning. Also makes sure that motor commands stay within range.
		// NOTE: THIS SHOULD BE UNNECESSARY. I IMPLEMENTED THIS AS AN EXTRA
		// SAFETY MEASURE
		if (cmd[i] < 1.0) {
			cmd[i] = 1.0;
		}
		else if (cmd[i] > 200.0) {
			cmd[i] = 200.0;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////

	/////// Send Motor Commands ///////////
	WO_Direct_Individual_Motor_Control.motor[0] = cmd[0];
	WO_Direct_Individual_Motor_Control.motor[3] = cmd[1];
	WO_Direct_Individual_Motor_Control.motor[1] = cmd[2];
	WO_Direct_Individual_Motor_Control.motor[2] = cmd[3];
	///////////////////////////////////////
}
/*-------------- End Command -------------*/


/*----------------------------------------------------------------------*/
/*---------------------------- End Helpers -----------------------------*/
/*----------------------------------------------------------------------*/
