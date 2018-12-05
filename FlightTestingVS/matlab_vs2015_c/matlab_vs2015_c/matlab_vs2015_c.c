/*
Simulator for AE483 course
--
Akshay Shetty and Dan Block
*/

//include header files
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"
#include "stdafx.h"
#include "planner.h"

//define structs for quadrotor and obstacle
PosYaw_f QuadDesired;
State6_f QuadCurrent;
State6_f ObstCurrent;

double x0[3] = { 0, 0, 1 };
double sim_time = 10;
double obst_init_pos[3] = { 1,0,.8 };
double obst_final_pos[3] = { 1,0,.8 };

int main()
{
	//initial position and orientation of quadrotor
	QuadCurrent.Pos.x = x0[0];	QuadCurrent.Pos.y = x0[1];	QuadCurrent.Pos.z = x0[2];
	QuadCurrent.Ori.tx = 0.0;	QuadCurrent.Ori.ty = 0.0;	QuadCurrent.Ori.tz = 0.0;

	//state vector used in simulator
	//x,y,z,vx,vy,vz,roll,pitch,yaw,roll_rate, pitch_rate,yaw_rate
	double state_x[12] = { 0.0 };
	//pointer to current state vector
	double *current_x;
	//set the initial values
	state_x[0] = QuadCurrent.Pos.x;	state_x[1] = QuadCurrent.Pos.y;	state_x[2] = QuadCurrent.Pos.z;
	state_x[6] = QuadCurrent.Ori.tx;	state_x[7] = QuadCurrent.Ori.ty;	state_x[8] = QuadCurrent.Ori.tz;

	//initialize desired position and yaw as 0s
	QuadDesired.Pos.x = 0.0;	QuadDesired.Pos.y = 0.0;	QuadDesired.Pos.z = 0.0;	QuadDesired.Tz = 0.0;
	double desired_x[4] = { QuadDesired.Pos.x, QuadDesired.Pos.y, QuadDesired.Pos.z, QuadDesired.Tz };

	//Obstacle details:
	/*
	Currently the simulator is set for spherical obstacles. The obstacle will move from obst_init_pos to obst_final_pos at a constant velocity from start to end of simulation
	*/
	double obst_s = 0.1;						//obstacle radius to be displayed on simulation
	double obst_curr_pos[3];
	obst_curr_pos[0] = obst_init_pos[0];	obst_curr_pos[1] = obst_init_pos[1];	obst_curr_pos[2] = obst_init_pos[2];
	ObstCurrent.Pos.x = obst_curr_pos[0];	ObstCurrent.Pos.y = obst_curr_pos[1];	ObstCurrent.Pos.z = obst_curr_pos[2];

	//define the time parameters here
	double timer = 0.0;
	double t_vector[3] = { 0.0, 0.02, sim_time };		//start time, time step, end time

	double percent = 0.0;

	//initialize MATLAB engine to call from within C
	Engine *ep;
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}
	
	//define mxArrays to send/receive variables with MATLAB
	mxArray *state_vector = NULL, *result = NULL, *t_info = NULL, *state_des = NULL, *obst_posn = NULL, *obst_rad = NULL;

	//addpath Quad_Sim to MATLAB engine

	engEvalString(ep, "addpath('Quad_Sim\')");  // Need to copy Quad_Sim directory to c:\matlab2018a\bin\win64
	
	//add state_x, t_vector, desired_x and obst_curr_pos to MATLAB workspace
	state_vector = mxCreateDoubleMatrix(12, 1, mxREAL);
	memcpy((void *)mxGetPr(state_vector), (void *)state_x, sizeof(state_x));
	engPutVariable(ep, "state_x", state_vector);

	t_info = mxCreateDoubleMatrix(1, 3, mxREAL);
	memcpy((void *)mxGetPr(t_info), (void *)t_vector, sizeof(t_vector));
	engPutVariable(ep, "t_vector", t_info);
	
	state_des = mxCreateDoubleMatrix(4, 1, mxREAL);
	memcpy((void *)mxGetPr(state_des), (void *)desired_x, sizeof(desired_x));
	engPutVariable(ep, "desired_x", state_des);

	obst_posn = mxCreateDoubleMatrix(1, 3, mxREAL);
	memcpy((void *)mxGetPr(obst_posn), (void *)obst_curr_pos, sizeof(obst_curr_pos));
	engPutVariable(ep, "obst_curr_pos", obst_posn);

	//in MATLAB, initialize the following variables:
	/*
	X_all: to record all the state vectors
	T_all: to record all the time values
	obst_all: to record all the obstacle positions
	des_all: to record all the desired positions generated
	timer: to keep track of running simulation time
	*/
	engEvalString(ep, "X_all = nan(round(t_vector(3)/t_vector(2))+1, 12);");
	engEvalString(ep, "T_all = nan(round(t_vector(3)/t_vector(2))+1, 1);");
	engEvalString(ep, "obst_all = nan(round(t_vector(3)/t_vector(2))+1, 3);");
	engEvalString(ep, "des_all = nan(round(t_vector(3)/t_vector(2))+1, 4);");
	engEvalString(ep, "timer = 0;");

	//define global flag in MATLAB to check when new data is sent over to the MATLAB engine
	engEvalString(ep, "global new_check;");
	engEvalString(ep, "new_check = 1;");

	//set initial values for state vector, time, obstacle positon and desired position
	engEvalString(ep, "X_all(1,:) = state_x';");
	engEvalString(ep, "T_all(1) = t_vector(1);");
	engEvalString(ep, "obst_all(1,:) = obst_curr_pos;");
	engEvalString(ep, "des_all(1,:) = desired_x';");

	//while simulation time is less than or equal to end time
	while (timer <= t_vector[2])
	{
		//calculate and display percent of simulation done
		percent = 100 * (timer / t_vector[2]);
		printf("Loading Simulation: %0.1f%% \n", percent);

		//call the planner.c function with the current quadrotor and obstacle position, and update the desired quadrotor position
		PotentialField(QuadCurrent, ObstCurrent, &QuadDesired, timer, 1);

		//put the desired position and angles in the MATLAB workspace
		desired_x[0] = QuadDesired.Pos.x;		desired_x[1] = QuadDesired.Pos.y;		desired_x[2] = QuadDesired.Pos.z;		desired_x[3] = QuadDesired.Tz;
		memcpy((void *)mxGetPr(state_des), (void *)desired_x, sizeof(desired_x));
		engPutVariable(ep, "desired_x", state_des);

		//call the Quad_Sim simulate_this and get the updated states in QuadCurrent struct
		engEvalString(ep, "simulate_this");
		engEvalString(ep, "new_check = 1;");
		engEvalString(ep, "state_x = X(end,:)'");
		result = engGetVariable(ep, "state_x");
		current_x = mxGetData(result);
		QuadCurrent.Pos.x = current_x[0];	QuadCurrent.Pos.y = current_x[1];	QuadCurrent.Pos.z = current_x[2];
		QuadCurrent.Ori.tx = current_x[6];	QuadCurrent.Ori.ty = current_x[7];	QuadCurrent.Ori.tz = current_x[8];
		
		//update simulation time by time step
		engEvalString(ep, "timer = timer + t_vector(2);");
		timer = timer + t_vector[1];

		//record all the required variables in MATLAB
		engEvalString(ep, "X_all(round(timer/t_vector(2))+1,:) = state_x'");
		engEvalString(ep, "T_all(round(timer/t_vector(2))+1) = timer;");
		engEvalString(ep, "des_all(round(timer/t_vector(2))+1,:) = desired_x'");

		//update obstacle position and put obst_curr_pos in MATLAB engine
		obst_curr_pos[0] = obst_init_pos[0] + (timer / t_vector[2])*(obst_final_pos[0] - obst_init_pos[0]);
		obst_curr_pos[1] = obst_init_pos[1] + (timer / t_vector[2])*(obst_final_pos[1] - obst_init_pos[1]);
		obst_curr_pos[2] = obst_init_pos[2] + (timer / t_vector[2])*(obst_final_pos[2] - obst_init_pos[2]);
		/*float vector[3];
		vector[0] = current_x[0] - obst_curr_pos[0];
		vector[1] = current_x[1] - obst_curr_pos[1];
		vector[2] = current_x[2] - obst_curr_pos[2];
		float norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
		obst_curr_pos[0] = obst_init_pos[0] + (timer / t_vector[2])*(current_x[0] - obst_init_pos[0]) - .5/norm*(current_x[0] - obst_curr_pos[0]);
		obst_curr_pos[1] = obst_init_pos[1] + (timer / t_vector[2])*(current_x[1] - obst_init_pos[1]) - .5/norm*(current_x[1] - obst_curr_pos[1]);
		obst_curr_pos[2] = obst_init_pos[2] + (timer / t_vector[2])*(current_x[2] - obst_init_pos[2]) - .5/norm*(current_x[2] - obst_curr_pos[2]);*/
		ObstCurrent.Pos.x = obst_curr_pos[0];	ObstCurrent.Pos.y = obst_curr_pos[1];	ObstCurrent.Pos.z = obst_curr_pos[2];
		memcpy((void *)mxGetPr(obst_posn), (void *)obst_curr_pos, sizeof(obst_curr_pos));
		engPutVariable(ep, "obst_curr_pos", obst_posn);
		engEvalString(ep, "obst_all(round(timer/t_vector(2))+1,:) = obst_curr_pos;");
	}

	engEvalString(ep, "X = X_all;");
	engEvalString(ep, "T = T_all;");
	
	//store values in mocap in the MATLAB workspace
	engEvalString(ep, "mocap = nan(7,length(T));");
	engEvalString(ep, "mocap(1,:) = T;");
	engEvalString(ep, "mocap(2:4,:) = X(:,1:3)';");
	engEvalString(ep, "mocap(5:7,:) = X(:,7:9)';");
	engEvalString(ep, "clearvars -except mocap obst_all des_all;");

	//send obstacle plot radius over to MATLAB workspace
	obst_rad = mxCreateDoubleScalar(obst_s);
	engPutVariable(ep, "obst_s", obst_rad);

	//make an animation with the stored states
	engEvalString(ep, "lab3_drawquad;");

	//Hit enter to close MATLAB engine
	printf("\nHit return to continue\n");
	fgetc(stdin);
	engClose(ep);

	//close Simulator
	return EXIT_SUCCESS;
}