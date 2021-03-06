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



//trial one
//float K_d = 1;
//float K_p = 1.4;
//float K_i[8] = { -3.137357986974831,  -0.559574426346653 ,  5.934718324391574, -11.057268646519193,
//		   0.070993619635960 ,  0.996049674973296 , -3.124855672533517,  -3.362203976500002};



//trial two and three(with %5)
/*
float K_d = 2.56;
float K_p = 2.24;
float K_i[8] = {-2.1894  ,  0.0323  ,  2.7697 ,  -5.3631,
	            0.0141,    2.1881  , -5.3690 ,  -2.7090};
*/
// Trial 3 Different Kd Kp
float K_d = 1;
float K_p = 1.40;
float K_i[8] = {-8.2235 ,   0.0974  ,  4.6412,  -36.0130,
				0.0827 ,   8.1586 , -35.7447,   -4.5133};


//-6.5 whatever
////////////////////////////////////////////


// Other variables you'll need to work with
float cnt_u[4];
float o_desired[3];
float v_desired[3] = {0,0,0};
float yaw_desired;
float g = 9.81;	// Standard Gravity m/s^2
float xd[12];   // discrete state (i.e., state error)
float xe[12];

float fsum_des;
float n_des[3];
// Other variables you can ignore for now
float Winv[4*4];
float omega_cmd2[4];
float omega_cmd[4];
float cmd[4];
int outter_count=0;
float epsilon_eq=0.039188;
float nx_eq=-0.105822;
float ny_eq=0.000000 ;
float nz_eq=0.994385 ;
float p_eq=-2.700386 ;
float q_eq=0.000000 ;
float r_eq=25.374955 ;
float  pd_eq=0.000000 ;
float qd_eq=0.000000 ;
float rd_eq=0.000000 ;
float w1_eq=614.993890 ;
float w2_eq=614.993890 ;
float w3_eq=434.866350 ;
float w4_eq=0.000000;
float se[4] = {-2.700386, 0, -0.105822, 0};
float ue[2] = {0, (7.46e-6)*(434.866350)*(434.866350)};
float a_desired[3] = {0,0,0};



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
	//float v_desired[0] = 0;
	//float v_desired[1] = 0;
	//float v_desired[2] = 0;
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




    xd[0] = mocap.dX;
    xd[1] = mocap.dY;
    xd[2] = mocap.dZ;
    xd[3] =  mocap.dThetaz;
    xd[4] =  imusensor.dThetay;
    xd[5] =  imusensor.dThetax;
    xd[6] = mocap.dVx;
    xd[7] = mocap.dVy;
    xd[8] = mocap.dVz;
    xd[9] = imusensor.dOmegax;
    xd[10] = imusensor.dOmegay;
    xd[11] = imusensor.dOmegaz;

    xe[0] = goalposyaw.x;
	xe[1] = goalposyaw.y;
	xe[2] = goalposyaw.z;
	xe[3] = 0.0;
	xe[4] = 0.0;
	xe[5] = 0.0;
	xe[6] = 0.0;
	xe[7] = 0.0;
	xe[8] = 0.0;
	xe[9] = p_eq;
	xe[10] = q_eq;
	xe[11] = r_eq;



    //////////////////////////////////////////////////////
    // Start the outer loop this defines 2 parameters from
    // state variables that the inner loop seeks to match.
    //
    // Target orientation vector defined in
    // room frame and rotated to body frame:
    // n_des = [nx ny nz];
    //
    // Target total force:
    // fsum_des
    //
    // Pass to inner loop to control the quadcopter
    //
    //////////////////////////////////////////////////////
    outter_count = outter_count+1;
if (outter_count%10==0) {

    a_desired[0] = -K_d*(xd[6]-xe[6]) -K_p*(xd[0]-xe[0]);
	a_desired[1] = -K_d*(xd[7]-xe[7]) -K_p*(xd[1]-xe[1]);
	a_desired[2] = -K_d*(xd[8]-xe[8]) -K_p*(xd[2]-xe[2]);


	float th1 = xd[3];
	float th2 = xd[4];
	float th3 = xd[5];

	float Rinv[9];
	/*
	Rinv[0] = (cos(xd[3])*cos(xd[4]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]));
	Rinv[1] = (cos(xd[4])*sin(xd[3]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]));
	Rinv[2] = -sin(xd[4])/(cos(xd[4])*cos(xd[4]) + sin(xd[4])*sin(xd[4]));
	Rinv[3] = -(cos(xd[4])*cos(xd[4])*cos(xd[5])*sin(xd[3]) - cos(xd[3])*sin(xd[4])*sin(xd[5]) + cos(xd[5])*sin(xd[3])*sin(xd[4])*sin(xd[4]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	Rinv[4] = (cos(xd[3])*cos(xd[5])*cos(xd[4])*cos(xd[4]) + cos(xd[3])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[5])*sin(xd[4]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	Rinv[5] = (cos(xd[4])*sin(xd[5]))/(cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	Rinv[6] = (sin(xd[3])*sin(xd[5])*cos(xd[4])*cos(xd[4]) + sin(xd[3])*sin(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[5])*sin(xd[4]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	Rinv[7] = -(cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5]) - cos(xd[5])*sin(xd[3])*sin(xd[4]) + cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	Rinv[8] = -(cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5]) - cos(xd[5])*sin(xd[3])*sin(xd[4]) + cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5]))/(cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[4])*cos(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[3])*cos(xd[3])*cos(xd[5])*cos(xd[5])*sin(xd[4])*sin(xd[4]) + cos(xd[3])*cos(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]) + cos(xd[4])*cos(xd[4])*cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3]) + cos(xd[4])*cos(xd[4])*sin(xd[3])*sin(xd[3])*sin(xd[5])*sin(xd[5]) + cos(xd[5])*cos(xd[5])*sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4]) + sin(xd[3])*sin(xd[3])*sin(xd[4])*sin(xd[4])*sin(xd[5])*sin(xd[5]));
	*/
	/*
	Rinv[0] = (cos(th1)*cos(th2))/(cos(th1)*cos(th1)*cos(th2)*cos(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2) + cos(th2)*cos(th2)*sin(th1)*sin(th1) + sin(th1)*sin(th1)*sin(th2)*sin(th2));
	Rinv[1] = (cos(th2)*sin(th1))/(cos(th1)*cos(th1)*cos(th2)*cos(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2) + cos(th2)*cos(th2)*sin(th1)*sin(th1) + sin(th1)*sin(th1)*sin(th2)*sin(th2));
	Rinv[2] = -sin(th2)/(cos(th2)*cos(th2) + sin(th2)*sin(th2));
	Rinv[3] = -(cos(th2)*cos(th2)*cos(th3)*sin(th1) - cos(th1)*sin(th2)*sin(th3) + cos(th3)*sin(th1)*sin(th2)*sin(th2))/(cos(th1)*cos(th1)*cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th1)*cos(th1)*cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th1)*cos(th1)*cos(th3)*cos(th3)*sin(th2)*sin(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3) + cos(th2)*cos(th2)*cos(th3)*cos(th3)*sin(th1)*sin(th1) + cos(th2)*cos(th2)*sin(th1)*sin(th1)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th1)*sin(th1)*sin(th2)*sin(th2) + sin(th1)*sin(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3));
	Rinv[4] = (cos(th1)*cos(th3)*cos(th2)*cos(th2) + cos(th1)*cos(th3)*sin(th2)*sin(th2) + sin(th1)*sin(th3)*sin(th2))/(cos(th1)*cos(th1)*cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th1)*cos(th1)*cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th1)*cos(th1)*cos(th3)*cos(th3)*sin(th2)*sin(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3) + cos(th2)*cos(th2)*cos(th3)*cos(th3)*sin(th1)*sin(th1) + cos(th2)*cos(th2)*sin(th1)*sin(th1)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th1)*sin(th1)*sin(th2)*sin(th2) + sin(th1)*sin(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3));
	Rinv[5] = (cos(th2)*sin(th3))/(cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th2)*sin(th2) + sin(th2)*sin(th2)*sin(th3)*sin(th3));
	Rinv[6] =  (sin(th1)*sin(th3)*cos(th2)*cos(th2) + sin(th1)*sin(th3)*sin(th2)*sin(th2) + cos(th1)*cos(th3)*sin(th2))/(cos(th1)*cos(th1)*cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th1)*cos(th1)*cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th1)*cos(th1)*cos(th3)*cos(th3)*sin(th2)*sin(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3) + cos(th2)*cos(th2)*cos(th3)*cos(th3)*sin(th1)*sin(th1) + cos(th2)*cos(th2)*sin(th1)*sin(th1)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th1)*sin(th1)*sin(th2)*sin(th2) + sin(th1)*sin(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3));
	Rinv[7] = -(cos(th1)*cos(th2)*cos(th2)*sin(th3) - cos(th3)*sin(th1)*sin(th2) + cos(th1)*sin(th2)*sin(th2)*sin(th3))/(cos(th1)*cos(th1)*cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th1)*cos(th1)*cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th1)*cos(th1)*cos(th3)*cos(th3)*sin(th2)*sin(th2) + cos(th1)*cos(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3) + cos(th2)*cos(th2)*cos(th3)*cos(th3)*sin(th1)*sin(th1) + cos(th2)*cos(th2)*sin(th1)*sin(th1)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th1)*sin(th1)*sin(th2)*sin(th2) + sin(th1)*sin(th1)*sin(th2)*sin(th2)*sin(th3)*sin(th3));
	Rinv[8] = (cos(th2)*cos(th3))/(cos(th2)*cos(th2)*cos(th3)*cos(th3) + cos(th2)*cos(th2)*sin(th3)*sin(th3) + cos(th3)*cos(th3)*sin(th2)*sin(th2) + sin(th2)*sin(th2)*sin(th3)*sin(th3));
	*/

	Rinv[0] = cos(th1)*cos(th2);
	Rinv[1] = cos(th2)*sin(th1);
	Rinv[2] = -sin(th2);
	Rinv[3] = cos(th1)*sin(th2)*sin(th3) - cos(th3)*sin(th1);
	Rinv[4] = cos(th1)*cos(th3) + sin(th1)*sin(th2)*sin(th3);
	Rinv[5] = cos(th2)*sin(th3);
	Rinv[6] = sin(th1)*sin(th3) + cos(th1)*cos(th3)*sin(th2);
	Rinv[7] = cos(th3)*sin(th1)*sin(th2) - cos(th1)*sin(th3);
	Rinv[8] = cos(th2)*cos(th3);

	float tmp_result[3] = {0,0,0};
	float a_desired_temp[3] = {a_desired[0],a_desired[1],a_desired[2]-g};
	matrix_multiply(3,3,1,Rinv,a_desired_temp,tmp_result);
	fsum_des = mass/nz_eq * sqrt(tmp_result[0]*tmp_result[0] + tmp_result[1]*tmp_result[1] + tmp_result[2]*tmp_result[2]);

	float n_des_temp[3];
	matrix_multiply(3,3,1,Rinv,a_desired_temp,n_des_temp);
	n_des[0] = n_des_temp[0] *mass/(nz_eq*fsum_des);
	n_des[1] = n_des_temp[1] *mass/(nz_eq*fsum_des);
	n_des[2] = n_des_temp[2] *mass/(nz_eq*fsum_des);
}

	/////////////////////////////////
	// Begin inner loop to match the target n_des and fsum_des.

	/////////////////////////////////
	se[2] = n_des[0];
	se[3] = n_des[1];


	float wB[3] = {xd[9], xd[10], xd[11]};
	float wBnorm = sqrt(wB[0]*wB[0] + wB[1]*wB[1] + wB[2]*wB[2]);

	float n[3] = {0, 0, -1};
	if(wBnorm > 0.000001)
	{
		n[0] = wB[0]/wBnorm;
		n[1] = wB[1]/wBnorm;
		n[2] = wB[2]/wBnorm;
	}


	float si[4] = {wB[0], wB[1], n[0], n[1]};
	float u_des_temp[2];
	float si_less_se[4];
	float u_desired[2];
	si_less_se[0] = si[0]-se[0];
	si_less_se[1] = si[1]-se[1];
	si_less_se[2] = si[2]-se[2];
	si_less_se[3] = si[3]-se[3];
	matrix_multiply(2,4,1,K_i,si_less_se,u_des_temp);
	u_desired[0] = -u_des_temp[0] + ue[0];
	u_desired[1] = -u_des_temp[1] + ue[1];
	float f2 = u_desired[1];
	float f3 = 1/2 * (fsum_des + u_desired[0] - f3);
	float f1 = fsum_des - f3 - f2;

	if(f1 < 0)
	{
		f1 = 0;
	}
	if(f2 < 0)
	{
		f2 = 0;
	}
	if(f3 < 0)
	{
		f3 = 0;
	}

	float w1sqrd = f1/kF;
	float w2sqrd = f2/kF;
	float w3sqrd = f3/kF;

	float sigmasqrd[4] = {w1sqrd,w2sqrd,w3sqrd,0};
	float Wmatrix[16] = {l*kF, -l*kF, 0, 0, 0, 0, l*kF, -l*kF, kM, kM, -kM, -kM, kF, kF, kF, kF};


	matrix_multiply(4,4,1,Wmatrix,sigmasqrd,cnt_u);

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
		//else if (omega_cmd2[i] < MINPHI2) {
		//	omega_cmd2[i] = MINPHI2;
		//}
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
			if(i==3)
			{
				cmd[i] = 0;
			}
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
