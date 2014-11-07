//
// move_sky_jens.cpp
// 
// Michael Mistry
// Disney Research Pittsburgh
// July 19, 2011
//
// Basic example of how to move joints and record positions
//
// Modified by Katsu Yamane
// December 5, 2011
// Collect data for system identification
//

//#include <iostream>
//#include <fstream>
#include <string>
#include <cstdio>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#include <vector>

using namespace std;

// Disney headers                                                                                                                                         
#include "DCSTYPES.H"
#include "msgtypes.h"
#include "DRP_sky_common.h"
#include "SPC_interface.h"

//#define LOOP_PERIOD_usec 33333 //30Hz
#define LOOP_PERIOD_usec 8333 //120Hz

#define USEC_TO_SEC(x) (x/1000000.0)
#define SEC_TO_USEC(x) (x*1000000.0)

char delim[] = " ,\n\t";

int main (int argc, char* argv[])
{

	// timing stuff
	struct timeval start, end;
	long seconds;
	long useconds;
	long time_usec = 0, loop_time = -9999999;

	ushort sin1, sin2;

	// data to save 
	//std::vector< std::vector<int> > input_data;  // actual input data sent to the joints
	std::vector<int> input_data; // actual input data sent to the joints
	std::vector<int> output_data; // measured data
	std::vector<long> time_data;




	// parameters needed to be specified every time
	std::string save_file_name = "";
	std::string dir_name = "results/";

	if(argc < 2) {
	  printf("usage: save <save_file_name>\n");
	  return -1;
	}


	std::string input_file_name;   // save input commands
	std::string output_file_name;  // save output (measurements)

	save_file_name = argv[1];
	input_file_name = dir_name + save_file_name + ".in.dat";
	output_file_name = dir_name + save_file_name + ".out.dat";

	printf("save_file_name = %s\n", save_file_name.c_str());

	//ushort joint_state[N_DOFS+1] = {0};      //actual position
	//ushort fsm_command[N_DOFS+1] = {0};      //current postion command as known by FSM

	// Initialization
	SPC_interface* spc = new SPC_interface;
	spc->initCommunication();



	//spc->printAllState();

	// Set FSMs to auto (can also do this via MIU)
	//for(int i=1; i<=N_DOFS; i++) {
	//  spc->setAuto(i);
	//}

	spc->setAuto(M);
        spc->setAuto(HN);
        spc->setAuto(HT);
        spc->setAuto(HTLT);
        spc->setAuto(ET);
        
        spc->setAuto(TF);
        spc->setAuto(TS);
        spc->setAuto(TT);
        spc->setAuto(P);
        spc->setAuto(BS);
        spc->setAuto(BF);
        spc->setAuto(BT);

        spc->setAuto(RSS);
        spc->setAuto(LSS);
        
        spc->setAuto(LAF);
        spc->setAuto(LAO);
        spc->setAuto(LAS);
        spc->setAuto(LE);
	
	spc->setAuto(LWIO);
        spc->setAuto(LWFB);
        spc->setAuto(LWT);

        spc->setAuto(LT);
        spc->setAuto(LFF);
        spc->setAuto(LMF);
        spc->setAuto(LRF);
        spc->setAuto(LLF);

        spc->setAuto(RAF);
        spc->setAuto(RAO);
        spc->setAuto(RAS);
        spc->setAuto(RE);

        spc->setAuto(RWIO);
        spc->setAuto(RWFB);
        spc->setAuto(RWT);
	
	spc->setAuto(RT);
        spc->setAuto(RFF);
        spc->setAuto(RMF);
        spc->setAuto(RRF);
        spc->setAuto(RLF);
	
	printf("finished setting auto\n");


	// Alex
	// joint state/command variables
	printf("DOF: %i\n", N_DOFS);
	double joint_des_state_double[N_DOFS+1] = {0};  //desired position in float
	ushort joint_des_state[N_DOFS+1] = {0};  //desired position in ushort
	ushort joint_state[N_DOFS+1] = {0};      //actual position
	ushort init_fsm_command[N_DOFS+1] = {0};      //initial postion command as known by FSM
	ushort fsm_command[N_DOFS+1] = {0};      //current postion command as known by FSM


	// intialize all desired positions so robot doesn't jump
	for(int i=1; i<=N_DOFS; i++) 
	{
		fsm_command[i] = spc->get_fsm_command(i);
		joint_state[i] = spc->get_joint_state(i);
		joint_des_state_double[i] = (double)fsm_command[i];
		joint_des_state[i] = fsm_command[i];
		init_fsm_command[i] = fsm_command[i];
	}


	for(int i=1; i<=N_DOFS; i++) 
	{
		joint_des_state[i] = 2000;
	}
	spc->setDesiredPosAlien(joint_des_state);


	gettimeofday(&start,NULL);
	gettimeofday(&end,NULL); 
	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	time_usec = (SEC_TO_USEC(seconds) + useconds);


	// run
	int loop_secs = 6;
	int loop_count = 0;

	while(time_usec <= SEC_TO_USEC(loop_secs))
	{

		gettimeofday(&end, NULL);
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		time_usec = (SEC_TO_USEC(seconds) + useconds);
		
		sin1 = 2048+1024*sin(M_PI*time_usec / 1000000.0);
		linear = 2048+1024*(time_usec - loop_secs/4)/ (1000000.0 * loop_secs/4);

		qua_var = (time_usec - loop_secs/2)/1000000.0;
		qua_time = (loop_secs/4)*(loop_secs/4);
		quadratic = 2048+1024*qua_var*qua_var/qua_time;

	    	// the iteration loop    
	    	if ( (time_usec - loop_time) > LOOP_PERIOD_usec) {
	      		loop_time = time_usec;

	      		if time_usec <= USEC_TO_SEC(loop_secs/4): 
	     		// compute sin wave stuff
	      		
					joint_des_state[RAF] = sin1;
					joint_des_state[RAO] = sin1;
					joint_des_state[RAS] = sin1;
					joint_des_state[RE] = sin1;
				else if (time_usec > USEC_TO_SEC(loop_secs/4) && time_usec <= USEC_TO_SEC(loop_secs/2)): 
					joint_des_state[RAF] = linear;
					joint_des_state[RAO] = linear;
					joint_des_state[RAS] = linear;
					joint_des_state[RE] = linear;
				else if (time_usec > USEC_TO_SEC(loop_secs/2) && time_usec <= USEC_TO_SEC(3*loop_secs/4)):
					joint_des_state[RAF] = quadratic;
					joint_des_state[RAO] = quadratic;
					joint_des_state[RAS] = quadratic;
					joint_des_state[RE] = quadratic;
				else if (time_usec > USEC_TO_SEC(3*loop_secs/4) && time_usec <= USEC_TO_SEC(loop_secs)): 
					joint_des_state[RAF] = sin1;
					joint_des_state[RAO] = sin1;
					joint_des_state[RAS] = sin1;
					joint_des_state[RE] = sin1;

			// Threshold
			short temp = round(joint_des_state[RE]);
			if (temp<0)
			{
				joint_des_state[RE] = 0;
			}
			else if (temp>4095)
			{
				joint_des_state[RE] = 4095;
			}
			else
			{
				joint_des_state[RE] = temp;
			}

			//printf("%u\n", joint_des_state[RE]);
			spc->setDesiredPosAlien(joint_des_state);

			for(int i=1; i<=N_DOFS; i++) 
			{
				fsm_command[i] = spc->get_fsm_command(i);
				joint_state[i] = spc->get_joint_state(i);
				input_data.push_back(fsm_command[i]);
				output_data.push_back(joint_state[i]);
			}

			time_data.push_back(time_usec);

		}


	
		loop_count++;


	}



	//spc->printAllState();

	// capture all FSMs that are in manual mode (also will intialize joint_state array for all FSMs)
	// note: spc->setPos() must be called within 100 ms or so, or capture will be lost
	//	spc->captureManual(joint_state, fsm_command);
	//	spc->printAllState();




	// Print Command (Input) Values


	if(input_data.size() > 0)
	{
		printf("Save Command Values to %s ... ", input_file_name.c_str());
		FILE* save_file = fopen(input_file_name.c_str(), "w");

		fprintf(save_file,"t_us\t");
		for(int i=1; i<=N_DOFS; i++) {
			fprintf(save_file, "%s\t", function_code_str[i]);
		}
		fprintf(save_file,"\n");

		for(int i=0; i<time_data.size(); i++)
		{
	
			fprintf(save_file, "%li\t", time_data[i]);
			
			for(int j=0; j<N_DOFS; j++) {
				fprintf(save_file, "%li\t", input_data[i*N_DOFS+j]);
			}
			
			fprintf(save_file,"\n");
		}

		fclose(save_file);
		printf("done\n");
	}


	if(output_data.size() > 0)
	{
		printf("Save Output Values to %s ... ", output_file_name.c_str());
		FILE* save_file = fopen(output_file_name.c_str(), "w");

		fprintf(save_file,"t_us\t");
		for(int i=1; i<=N_DOFS; i++) {
			fprintf(save_file, "%s\t", function_code_str[i]);
		}
		fprintf(save_file,"\n");

		for(int i=0; i<time_data.size(); i++)
		{
	
			fprintf(save_file, "%li\t", time_data[i]);
			
			for(int j=0; j<N_DOFS; j++) {
				fprintf(save_file, "%li\t", output_data[i*N_DOFS+j]);
			}
			
			fprintf(save_file,"\n");
		}

		fclose(save_file);
		printf("done\n");
	}







	int dof_start = RAF; // 1;
	int dof_end = RE; // N_DOFS;

	for(int i=dof_start; i<=dof_end; i++) 
	{
		fsm_command[i] = spc->get_fsm_command(i);
		joint_state[i] = spc->get_joint_state(i);
	}

	// save fsm_command to a file
	printf("save current command pose to %s ... ", save_file_name.c_str());
	FILE* save_file = fopen(save_file_name.c_str(), "w");
	//for(int i=1; i<=N_DOFS; i++) {
	//  fprintf(save_file, "%s %u\n", function_code_str[i], fsm_command[i]);
	//}

	for(int i=dof_start; i<=dof_end; i++) {
		fprintf(save_file, "%s_d\t", function_code_str[i]);
	}
	for(int i=dof_start; i<=dof_end; i++) {
		fprintf(save_file, "%s\t", function_code_str[i]);
	}
	fprintf(save_file, "\n");

	for(int i=dof_start; i<=dof_end; i++) {
		fprintf(save_file, "%u\t", fsm_command[i]);
	}
	for(int i=dof_start; i<=dof_end; i++) {
		fprintf(save_file, "%u\t", joint_state[i]);
	}
	fprintf(save_file, "\n");		

	fclose(save_file);
	printf("done\n");

#if 0
	// bring back to auto
	for(int i=1; i<=N_DOFS; i++) {
	  spc->setAuto(i);
	}
#endif

	delete spc;
	return 0;
}
