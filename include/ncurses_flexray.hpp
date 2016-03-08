#pragma once

#include "FlexRayHardwareInterface.hpp"
#include <ncurses.h>

enum COLORS{
	CYAN=1,
	RED,
	GREEN,
};

struct MotorData{
	float jointPos;
	float actuatorPos;
	float actuatorVel;
	uint16 actuatorCurrent;
	sint16 tendonDisplacement;
}motor;

//! standard query messages
char welcomestring[] = "commandline tool for controlling myode muscle via flexray ganglion setup";
char commandstring[] = "[0]position, [1]velocity, [2]force, [3]switch motor, [4]connection speed, [5]record, [6]allToForce ,[9]exit";
char setpointstring[] = "set point (rad) ?";
char setvelstring[] = "set velocity (rad/s) ?";
char setforcestring[] = "set force (N) ?";
char motorstring[] = "which motor(0-3)?";
char motorinfo[30] ;
char ganglionstring[] = "which ganglion(0-5)?";
char runningstring[] = "running ";
char recordingstring[] = "recording ";
char donestring[] = "done ";
char samplingtimestring [] = "samplingTime [milliseconds]: ";
char recordtimestring [] = "recordTime [seconds]: ";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char averageconnectionspeedstring[] = "average connection speed: ";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char byebyestring[] = "BYE BYE!";

class NCurses_flexray{
public:
	NCurses_flexray(){
		//! start ncurses mode
		initscr();
		//! Start color functionality
		start_color();
		init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
		init_pair(RED, COLOR_RED, COLOR_BLACK);
		init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
		//! get the size of the terminal window
		getmaxyx(stdscr,rows,cols);

		print(0,0,cols,"-");
		printMessage(1,0,welcomestring);
		print(2,0,cols,"-");
		print(6,0,cols,"-");
		querySensoryData();
		printMessage(3,0,commandstring);
	}
	~NCurses_flexray(){
		clearAll(0);
		printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring);
		refresh();
		usleep(1000000);
		endwin();
	}
	void printMessage(uint row, uint col, char* msg){
		mvprintw(row,col,"%s", msg);
		refresh();
	}
	void printMessage(uint row, uint col, char* msg, uint color){
		mvprintw(row,col,"%s", msg);
		mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
		refresh();
	}
	void print(uint row, uint startcol, uint length, const char* s){
		for (uint i=startcol;i<startcol+length;i++){
			mvprintw(row,i,"%s",s);
		}
		refresh();
	}
	void clearAll(uint row){
		for(uint i=row;i<rows;i++){
			print(i,0,cols," ");
		}
		refresh();
	}
	void querySensoryData(){
		flexray.exchangeData();
		motor.jointPos = flexray.GanglionData[ganglion_id].muscleState[motor_id].jointPos;
		motor.actuatorPos = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorPos*flexray.controlparams.radPerEncoderCount;
		motor.actuatorVel = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorVel*flexray.controlparams.radPerEncoderCount;
		motor.actuatorCurrent = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorCurrent;
		motor.tendonDisplacement = flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement;

		sprintf(motorinfo,"ganglion %d, motor %d   ", ganglion_id, motor_id);
		printMessage(7,0,motorinfo, CYAN);
		mvprintw(8,0,"actuatorPos (rad):   %.5f    ",motor.actuatorPos);
		mvprintw(9,0,"actuatorVel (rad/s): %.5f    ",motor.actuatorVel);
		mvprintw(10,0,"actuatorCurrent:     %d     ",motor.actuatorCurrent);
		mvprintw(11,0,"tendonDisplacement:  %.5f   ", (float)motor.tendonDisplacement/32768.0f);
		print(12,0,cols,"-");
		mvprintw(13,0,"P gain:          %.5f       ",flexray.controlparams.params.pidParameters.pgain);
		mvprintw(14,0,"I gain:          %.5f       ",flexray.controlparams.params.pidParameters.igain);
		mvprintw(15,0,"D gain:          %.5f       ",flexray.controlparams.params.pidParameters.dgain);
		mvprintw(16,0,"forward gain:    %.5f       ",flexray.controlparams.params.pidParameters.forwardGain);
		mvprintw(17,0,"deadband:        %.5f       ",flexray.controlparams.params.pidParameters.deadBand);
		if(ganglion_id<3)
			mvprintw(18,0,"set point:       %.5f   ",flexray.commandframe0[ganglion_id].sp[motor_id]);
		else
			mvprintw(18,0,"set point:       %.5f   ",flexray.commandframe1[ganglion_id].sp[motor_id]);
		print(19,0,cols,"-");
		mvprintw(20,0,"polyPar: %.5f  %.5f  %.5f  %.5f    ",flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0]);
		mvprintw(21,0,"set point limits: %.5f to %.5f     ",flexray.controlparams.spNegMax,flexray.controlparams.spPosMax);
		refresh();
	}
	void processing(char* msg1, char* what, char* msg2){
		char cmd;
		uint a = strlen(msg1);
		uint b = strlen(what);
		uint c = strlen(msg2);

		print(5,0,cols," ");
		printMessage(5,0,msg1);
		printMessage(5,a+1, what);
		printMessage(5,a+1+b+1, msg2);
		mvchgat(5, 0,       a+1+b, A_BLINK, 2, NULL);
		mvchgat(5, a+1+b+1, a+1+b+1+c, A_BLINK, 1, NULL);
		timeout(10);
		do{
			querySensoryData();
			cmd = mvgetch(5,a+1+b+1+c);
		}while(cmd != 'q');
		timeout(-1);
	}
	void processing(char* msg1, char* msg2){
		char cmd;
		uint a = strlen(msg1);
		uint c = strlen(msg2);

		print(5,0,cols," ");
		printMessage(5,0,msg1);
		printMessage(5,a+1, msg2);
		mvchgat(5, 0,       a, A_BLINK, 2, NULL);
		mvchgat(5, a+1, a+1+c, A_BLINK, 1, NULL);
		timeout(10);
		do{
			querySensoryData();
			cmd = mvgetch(5,a+1+c);
		}while(cmd != 'q');
		timeout(-1);
	}
	void positionControl(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		flexray.initPositionControl(ganglion_id,motor_id);
		printMessage(4,0,setpointstring);
		mvchgat(4, 0, strlen(setpointstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5,0,inputstring,30);
		pos = atof(inputstring);
		if(ganglion_id < 3)
			flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
		else
			flexray.commandframe1[ganglion_id-3].sp[motor_id] = pos;
		flexray.updateCommandFrame();
		flexray.exchangeData();
		processing(runningstring, inputstring, quitstring);
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
	void velocityControl(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		flexray.initVelocityControl(ganglion_id,motor_id);
		printMessage(4,0,setvelstring);
		mvchgat(4, 0, strlen(setvelstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5,0,inputstring,30);
		pos = atof(inputstring);
		if(ganglion_id < 3)
			flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
		else
			flexray.commandframe1[ganglion_id-3].sp[motor_id] = pos;
		flexray.updateCommandFrame();
		flexray.exchangeData();
		processing(runningstring, inputstring, quitstring);
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
	void forceControl(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		flexray.initForceControl(ganglion_id,motor_id);
		printMessage(4,0,setforcestring);
		mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5,0,inputstring,30);
		pos = atof(inputstring);
		if(ganglion_id < 3)
			flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
		else
			flexray.commandframe1[ganglion_id-3].sp[motor_id] = pos;
		flexray.updateCommandFrame();
		flexray.exchangeData();
		processing(runningstring, inputstring, quitstring);
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
	void switchMotor(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		printMessage(4,0,ganglionstring, GREEN);
		mvgetnstr(5,0,inputstring,30);
		uint ganlionrequest = atoi(inputstring);
		if(ganlionrequest<6)
			ganglion_id = ganlionrequest;
		else {
			print(4,0,cols," ");
			print(5,0,cols," ");
			printMessage(5, 0, invalidstring, RED);
			return;
		}
		print(4,0,cols," ");
		print(5,0,cols," ");
		printMessage(4,0,motorstring, GREEN);
		mvgetnstr(5,0,inputstring,30);
		uint motorrequest = atoi(inputstring);
		if(motorrequest<4)
			motor_id = motorrequest;
		else {
			print(4,0,cols," ");
			print(5,0,cols," ");
			printMessage(5, 0, invalidstring, RED);
			return;
		}
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
	void measureConnection(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		double averageTime = flexray.measureConnectionTime();
		printMessage(4, 0, averageconnectionspeedstring);
		char str[20];
		sprintf(str, "%f seconds", averageTime);
		printMessage(4,strlen(averageconnectionspeedstring),str, CYAN);
		printMessage(5,0,logfilestring, CYAN);
		usleep(5000000);
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
	void recordTrajectories(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		printMessage(4,0,filenamestring);
		mvgetnstr(4,strlen(filenamestring),inputstring,30);
		std::string name(inputstring);
		print(4,0,cols," ");
		printMessage(4, 0, samplingtimestring, CYAN);
		mvgetnstr(4,strlen(samplingtimestring),inputstring,30);
		float samplingTime = atof(inputstring);
		printMessage(5, 0, recordtimestring, CYAN);
		mvgetnstr(5,strlen(recordtimestring),inputstring,30);
		double recordTime = atof(inputstring);
		print(4,0,cols," ");
		print(5,0,cols," ");
		printMessage(4,0,recordingstring,RED);
		std::vector<std::vector<float>> trajectories;
		std::vector<int8_t> idList = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
		std::vector<uint8_t> controlmode(24,1);
		float averageSamplingTime = flexray.recordTrajectories(samplingTime,recordTime,trajectories,idList,controlmode,name);
		print(4,0,cols," ");
		printMessage(4,0,donestring,GREEN);
		char averagetimestring[50];
		sprintf(averagetimestring, "average %s%f", samplingtimestring, averageSamplingTime);
		printMessage(4,strlen(donestring),averagetimestring, CYAN);
		usleep(500000);
		print(4,0,cols," ");
		print(5,0,cols," ");
	}

	void setAllToForce(){
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");
		printMessage(4,0,setforcestring);
		mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5,0,inputstring,30);
		pos = atof(inputstring);
		for (uint ganglion=0;ganglion<NUMBER_OF_GANGLIONS;ganglion++){
			for (uint motor=0;motor<4;motor++){
				if(ganglion_id < 3)
					flexray.commandframe0[ganglion].sp[motor] = pos;
				else
					flexray.commandframe1[ganglion-3].sp[motor] = pos;
			}
		}
		flexray.initForceControl();
		processing(runningstring, inputstring, quitstring);
		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
private:
	FlexRayHardwareInterface flexray;
	uint rows, cols;
	float pos;
	uint ganglion_id=0;
	uint motor_id=0;
	char inputstring[30];
	MotorData motor;
};
