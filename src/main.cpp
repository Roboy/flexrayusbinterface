//#include "FlexRayUSBInterface.h"
#include "FlexRayHardwareInterface.hpp"

#include <ncurses.h>
#include <stdlib.h>     /* atof */

FlexRayHardwareInterface flexray;

struct MotorData{
    float jointPos;
    float actuatorPos;
    float actuatorVel;
    uint16 actuatorCurrent;
    sint16 tendonDisplacement;
}motor;

//! size of terminal window
uint rows,cols;

void printMessage(uint row, uint col, char* msg){
    mvprintw(row,col,"%s", msg);
}

void print(uint row, uint startcol, uint length, const char* s){
    for (uint i=startcol;i<startcol+length;i++){
        mvprintw(row,i,"%s",s);
    }
}

void clearAll(){
    for(uint i=0;i<rows;i++){
        print(i,0,cols," ");
    }
}

void querySensoryData(){
    flexray.exchangeData();
    motor.jointPos = flexray.GanglionData[0].muscleState[0].jointPos;
    motor.actuatorPos = flexray.GanglionData[0].muscleState[0].actuatorPos*flexray.controlparams.radPerEncoderCount;
    motor.actuatorVel = flexray.GanglionData[0].muscleState[0].actuatorVel*flexray.controlparams.radPerEncoderCount;
    motor.actuatorCurrent = flexray.GanglionData[0].muscleState[0].actuatorCurrent;
    motor.tendonDisplacement = flexray.GanglionData[0].muscleState[0].tendonDisplacement;
    
    mvprintw(7,0,"actuatorPos (rad):   %.5f",motor.actuatorPos);
    mvprintw(8,0,"actuatorVel (rad/s): %.5f",motor.actuatorVel);
    mvprintw(9,0,"actuatorCurrent:     %d",motor.actuatorCurrent);
    mvprintw(10,0,"tendonDisplacement:  %.5f", (float)motor.tendonDisplacement/32768.0f);
    print(11,0,cols,"-");
    mvprintw(12,0,"P gain:          %.5f",flexray.controlparams.params.pidParameters.pgain);
    mvprintw(13,0,"I gain:          %.5f",flexray.controlparams.params.pidParameters.igain);
    mvprintw(14,0,"D gain:          %.5f",flexray.controlparams.params.pidParameters.dgain);
    mvprintw(15,0,"forward gain:    %.5f",flexray.controlparams.params.pidParameters.forwardGain);
    mvprintw(16,0,"deadband:        %.5f",flexray.controlparams.params.pidParameters.deadBand);
    mvprintw(17,0,"set point:       %.5f",flexray.commandframe0[0].sp[0]);
    print(18,0,cols,"-");
    mvprintw(19,0,"polyPar: %.5f  %.5f  %.5f  %.5f",flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0],flexray.controlparams.polyPar[0]);
    mvprintw(20,0,"set point limits: %.5f to %.5f",flexray.controlparams.spNegMax,flexray.controlparams.spPosMax);
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

int main(int argc, char* argv[]){
    //! start ncurses mode
    initscr();
    //! Start color functionality	
    start_color();				
    init_pair(1, COLOR_CYAN, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    
    //! get the size of the terminal window
    getmaxyx(stdscr,rows,cols);    
    
    //! standard query messages
    char welcomestring[] = "commandline tool for controlling myode muscle via flexray ganglion setup";
    char commandstring[] = "[0]position, [1]velocity, [2]force, [3]monitor motor, [9]exit";
    char setpointstring[] = "set point (rad) ?";
    char setvelstring[] = "set velocity (rad/s) ?";
    char setforcestring[] = "set force (N) ?";
    char monitormotorstring[] = "monitoring motor data";
    char runningstring[] = "running ";
    char quitstring[] = " [hit q to quit]";
    char byebyestring[] = "BYE BYE!";
    
    print(0,0,cols,"-");
    printMessage(1,0,welcomestring);
    print(2,0,cols,"-");
    print(6,0,cols,"-");
    querySensoryData();
    printMessage(3,0,commandstring);
    
    char cmd;
    float pos;
    char floatstring[30];
    
    do{
        print(4,0,cols," ");
        print(5,0,cols," ");
        refresh();
        cmd = mvgetch(4,0);
        switch (cmd){
            case '0':
                flexray.initPositionControl();
                printMessage(4,0,setpointstring);
                mvchgat(4, 0, strlen(setpointstring), A_BOLD, 1, NULL);
                refresh();
                mvgetnstr(5,0,floatstring,30);
                pos = atof(floatstring);
                flexray.commandframe0[0].sp[0] = pos;
                flexray.updateCommandFrame();
                flexray.exchangeData();
                processing(runningstring, floatstring, quitstring);
                break;
            case '1':
                flexray.initVelocityControl();
                printMessage(4,0,setvelstring);
                mvchgat(4, 0, strlen(setvelstring), A_BOLD, 1, NULL);
                refresh();
                mvgetnstr(5,0,floatstring,30);
                pos = atof(floatstring);
                flexray.commandframe0[0].sp[0] = pos;
                flexray.updateCommandFrame();
                flexray.exchangeData();
                processing(runningstring, floatstring, quitstring);
                break;
            case '2':
                flexray.initForceControl();
                printMessage(4,0,setforcestring);
                mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
                refresh();
                mvgetnstr(5,0,floatstring,30);
                pos = atof(floatstring);
                flexray.commandframe0[0].sp[0] = pos;
                flexray.updateCommandFrame();
                flexray.exchangeData();
                processing(runningstring, floatstring, quitstring);
                break;
            case '3':
                processing(monitormotorstring,quitstring);
                break;
            case '9':
                clearAll();
                printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring);
                refresh();
                usleep(1000000);
        }
        
    }while( cmd != '9');
    endwin();
    return 0;
}
