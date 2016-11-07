#pragma once

#include "FlexRayHardwareInterface.hpp"
#include <ncurses.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "common_utilities/MotorStatus.h"
#include "common_utilities/MotorCommand.h"

enum COLORS {
    CYAN = 1,
    RED,
    GREEN,
};

struct MotorData {
    float jointPos;
    float actuatorPos;
    float actuatorVel;
    uint16 actuatorCurrent;
    sint16 tendonDisplacement;
} motor;


//! standard query messages
char welcomestring[] = "commandline tool for controlling myode muscle via flexray ganglion setup";
char commandstring[] = "[0]position, [1]velocity, [2]force, [3]switch motor, [4]connection speed, [5]record, [6]allToForce, [7] resettingSpring, [8] resettAll, [r]android, [p]publishMotorInfo, [9]exit";
char setpointstring[] = "set point (rad) ?";
char setvelstring[] = "set velocity (rad/s) ?";
char setforcestring[] = "set force (N) ?";
char motorstring[] = "which motor(0-3)?";
char motorinfo[30];
char ganglionstring[] = "which ganglion(0-5)?";
char runningstring[] = "running ";
char recordingstring[] = "recording ";
char donestring[] = "done ";
char samplingtimestring[] = "samplingTime [milliseconds]: ";
char recordtimestring[] = "recordTime [seconds]: ";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char averageconnectionspeedstring[] = "average connection speed: ";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char remotecontrolactivestring[] = "remote control active [hit q to quit]";
char publishingmotorstring[] = "publishing motor status[hit q to quit]";
char receivedupdatestring[] = "received update";
char errormessage[] = "Error: received update for motor that is not connected";
char byebyestring[] = "BYE BYE!";

class NCurses_flexray {
public:
    NCurses_flexray() {
        //! start ncurses mode
        initscr();
        //! Start color functionality
        start_color();
        init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
        init_pair(RED, COLOR_RED, COLOR_BLACK);
        init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
        //! get the size of the terminal window
        getmaxyx(stdscr, rows, cols);

        print(0, 0, cols, "-");
        printMessage(1, 0, welcomestring);
        print(2, 0, cols, "-");
        print(6, 0, cols, "-");
        querySensoryData();
        printMessage(3, 0, commandstring);

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "flexray_ncurse_interface", ros::init_options::NoSigintHandler);
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);

        spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
        spinner->start();
    }

    ~NCurses_flexray() {
        clearAll(0);
        printMessage(rows / 2, cols / 2 - strlen(byebyestring) / 2, byebyestring);
        refresh();
        usleep(1000000);
        endwin();
    }

    void printMessage(uint row, uint col, char *msg) {
        mvprintw(row, col, "%s", msg);
        refresh();
    }

    void printMessage(uint row, uint col, char *msg, uint color) {
        mvprintw(row, col, "%s", msg);
        mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
        refresh();
    }

    void print(uint row, uint startcol, uint length, const char *s) {
        for (uint i = startcol; i < startcol + length; i++) {
            mvprintw(row, i, "%s", s);
        }
        refresh();
    }

    void clearAll(uint row) {
        for (uint i = row; i < rows; i++) {
            print(i, 0, cols, " ");
        }
        refresh();
    }

    void querySensoryData() {
        flexray.exchangeData();
        motor.jointPos = flexray.GanglionData[ganglion_id].muscleState[motor_id].jointPos;
        motor.actuatorPos = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorPos *
                            flexray.controlparams.radPerEncoderCount;
        motor.actuatorVel = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorVel *
                            flexray.controlparams.radPerEncoderCount;
        motor.actuatorCurrent = flexray.GanglionData[ganglion_id].muscleState[motor_id].actuatorCurrent;
        motor.tendonDisplacement = flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement;

        sprintf(motorinfo, "ganglion %d, motor %d   ", ganglion_id, motor_id);
        printMessage(7, 0, motorinfo, CYAN);
        mvprintw(8, 0, "actuatorPos (rad):   %.5f    ", motor.actuatorPos);
        mvprintw(9, 0, "actuatorVel (rad/s): %.5f    ", motor.actuatorVel);
        mvprintw(10, 0, "actuatorCurrent:     %d     ", motor.actuatorCurrent);
        mvprintw(11, 0, "tendonDisplacement:  %.5f   ", (float) motor.tendonDisplacement / 32768.0f);
        print(12, 0, cols, "-");
        mvprintw(13, 0, "P gain:          %.5f       ", flexray.controlparams.params.pidParameters.pgain);
        mvprintw(14, 0, "I gain:          %.5f       ", flexray.controlparams.params.pidParameters.igain);
        mvprintw(15, 0, "D gain:          %.5f       ", flexray.controlparams.params.pidParameters.dgain);
        mvprintw(16, 0, "forward gain:    %.5f       ", flexray.controlparams.params.pidParameters.forwardGain);
        mvprintw(17, 0, "deadband:        %.5f       ", flexray.controlparams.params.pidParameters.deadBand);
        if (ganglion_id < 3)
            mvprintw(18, 0, "set point:       %.5f   ", flexray.commandframe0[ganglion_id].sp[motor_id]);
        else
            mvprintw(18, 0, "set point:       %.5f   ", flexray.commandframe1[ganglion_id].sp[motor_id]);
        print(19, 0, cols, "-");
        mvprintw(20, 0, "polyPar: %.5f  %.5f  %.5f  %.5f    ", flexray.controlparams.polyPar[0],
                 flexray.controlparams.polyPar[1], flexray.controlparams.polyPar[2], flexray.controlparams.polyPar[3]);
        mvprintw(21, 0, "set point limits: %.5f to %.5f     ", flexray.controlparams.spNegMax,
                 flexray.controlparams.spPosMax);
        refresh();
    }

    void processing(char *msg1, char *what, char *msg2) {
        char cmd;
        uint a = strlen(msg1);
        uint b = strlen(what);
        uint c = strlen(msg2);

        print(5, 0, cols, " ");
        printMessage(5, 0, msg1);
        printMessage(5, a + 1, what);
        printMessage(5, a + 1 + b + 1, msg2);
        mvchgat(5, 0, a + 1 + b, A_BLINK, 2, NULL);
        mvchgat(5, a + 1 + b + 1, a + 1 + b + 1 + c, A_BLINK, 1, NULL);
        timeout(10);
        do {
            querySensoryData();
            cmd = mvgetch(5, a + 1 + b + 1 + c);
        } while (cmd != 'q');
        timeout(-1);
    }

    void processing(char *msg1, char *msg2) {
        char cmd;
        uint a = strlen(msg1);
        uint c = strlen(msg2);

        print(5, 0, cols, " ");
        printMessage(5, 0, msg1);
        printMessage(5, a + 1, msg2);
        mvchgat(5, 0, a, A_BLINK, 2, NULL);
        mvchgat(5, a + 1, a + 1 + c, A_BLINK, 1, NULL);
        timeout(10);
        do {
            querySensoryData();
            cmd = mvgetch(5, a + 1 + c);
        } while (cmd != 'q');
        timeout(-1);
    }


    void resetControl() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        clearAll(23);
        flexray.initVelocityControl(ganglion_id, motor_id);
        uint count = 0;
        float aux = 0;
        uint p = 0;
        char inputspring[] = " running";
        float vel;
        float tendonDisplacement_t[3];
        float tendonDisplacement_t2[200000];
        sleep(1);

        pos = 0;
        if (ganglion_id < 3)
            flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
        else
            flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
        flexray.updateCommandFrame();
        flexray.exchangeData();
        querySensoryData();
        usleep(300000);
        flexray.updateCommandFrame();
        flexray.exchangeData();
        tendonDisplacement_t[0] = flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement /
                                  32768.0f; //tendon displacemnte iniziale
        //mvprintw(23,0,"Displacement_t0 %.5f 	",tendonDisplacement_t[0]);
        refresh();
        //Shaking the spring
        uint i;
        for (i = 1; i < 3; i++) {
            pos = 3;
            if (ganglion_id < 3)
                flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
            else
                flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
            flexray.updateCommandFrame();
            flexray.exchangeData();
            querySensoryData();
            usleep(500000);
            flexray.updateCommandFrame();
            flexray.exchangeData();
            tendonDisplacement_t[i] =
                    flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
            //mvprintw(23+i,0,"Displacement_t%d %.5f ",i,tendonDisplacement_t[i]);
            refresh();
        }
        //Relaxing the spring
        uint t = 0;
        tendonDisplacement_t2[0] =
                flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
        //mvprintw(23+i,0,"Displacement_t20 %.5f ",tendonDisplacement_t2[t]);
        refresh();

        if (tendonDisplacement_t[2] < tendonDisplacement_t[0])
            vel = 3;
        else if (tendonDisplacement_t[1] < tendonDisplacement_t[2])
            vel = -3;
        else
            vel = 0;

        do {
            pos = vel;
            if (ganglion_id < 3)
                flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
            else
                flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
            flexray.updateCommandFrame();
            flexray.exchangeData();
            querySensoryData();
            usleep(300000);
            flexray.updateCommandFrame();
            flexray.exchangeData();
            tendonDisplacement_t2[t + 1] =
                    flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
            t++;
            //mvprintw(23+i+t,0,"Displacement_t2 %d %.5f ",t,tendonDisplacement_t2[t]);
            refresh();

            if (tendonDisplacement_t2[t] > tendonDisplacement_t2[t - 1])
                p++;

            else if (tendonDisplacement_t2[t] == tendonDisplacement_t2[t - 1]) {


                if (count > 0 && tendonDisplacement_t2[t] == aux)
                    count++;
                else if (count == 0) {
                    aux = tendonDisplacement_t2[t];
                    count++;
                } else count = 0;
            }

        } while (count != 4 && p != 3);

        if (p == 3)
            resetControl();
        else {
            pos = 0;
            if (ganglion_id < 3)
                flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
            else
                flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
            flexray.updateCommandFrame();
            flexray.exchangeData();
            querySensoryData();
            sleep(2);
            refresh();

            //Resetting the sensor displacement
            flexray.controlparams.tag = 1;
            flexray.commandframe0[0].OperationMode[0] = Initialise;
            refresh();
            flexray.updateCommandFrame();
            flexray.exchangeData();
            flexray.updateMotorState();
            querySensoryData();
            flexray.controlparams.tag = 0;
            flexray.commandframe0[0].OperationMode[0] = Initialise;
            refresh();
            flexray.updateCommandFrame();
            flexray.exchangeData();
            flexray.updateMotorState();
            querySensoryData();
            mvprintw(22, 0, "Sensor Displacement Resetted");
            refresh();
            print(4, 0, cols, " ");
            print(5, 0, cols, " ");
            noecho();
        }
    }

    void resetAll() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        clearAll(23);
        flexray.initVelocityControl();
        uint s = 0;
        for (uint ganglion_id = 0; ganglion_id <
                                   NUMBER_OF_GANGLIONS; ganglion_id++) { //NUMBER_OF_GANGLIONS has to be adjusted to the number of ganglions actually connected
            for (uint motor_id = 0; motor_id < 4; motor_id++) {
                uint count = 0;
                float aux = 0;
                uint p = 0;
                char inputspring[] = " running";
                float vel;
                float tendonDisplacement_t[3];
                float tendonDisplacement_t2[200000];
                sleep(1);

                pos = 0;
                if (ganglion_id < 3)
                    flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
                else
                    flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
                flexray.updateCommandFrame();
                flexray.exchangeData();
                querySensoryData();
                usleep(300000);
                flexray.updateCommandFrame();
                flexray.exchangeData();
                tendonDisplacement_t[0] = flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement /
                                          32768.0f; //tendon displacemnte iniziale
                //mvprintw(23,0,"Displacement_t0 %.5f 	",tendonDisplacement_t[0]);
                refresh();

                //Shaking the spring
                uint i;
                for (i = 1; i < 3; i++) {
                    pos = 3;
                    if (ganglion_id < 3)
                        flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
                    else
                        flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    querySensoryData();
                    usleep(500000);
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    tendonDisplacement_t[i] =
                            flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
                    //mvprintw(23+i,0,"Displacement_t%d %.5f ",i,tendonDisplacement_t[i]);
                    refresh();
                }
                //relaxing the spring
                uint t = 0;
                tendonDisplacement_t2[0] =
                        flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
                //mvprintw(23+i,0,"Displacement_t20 %.5f ",tendonDisplacement_t2[t]);
                refresh();

                if (tendonDisplacement_t[2] < tendonDisplacement_t[0])
                    vel = 3;
                else if (tendonDisplacement_t[1] < tendonDisplacement_t[2])
                    vel = -3;
                else
                    vel = 0;

                do {
                    pos = vel;
                    if (ganglion_id < 3)
                        flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
                    else
                        flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    querySensoryData();
                    usleep(300000);
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    tendonDisplacement_t2[t + 1] =
                            flexray.GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f;
                    t++;
                    //mvprintw(23+i+t,0,"Displacement_t2 %d %.5f ",t,tendonDisplacement_t2[t]);
                    refresh();

                    if (tendonDisplacement_t2[t] > tendonDisplacement_t2[t - 1])
                        p++;

                    else if (tendonDisplacement_t2[t] == tendonDisplacement_t2[t - 1]) {


                        if (count > 0 && tendonDisplacement_t2[t] == aux)
                            count++;
                        else if (count == 0) {
                            aux = tendonDisplacement_t2[t];
                            count++;
                        } else count = 0;
                    }

                } while (count != 4 && p != 3);

                if (p == 3)
                    resetControl();
                else {
                    pos = 0;
                    if (ganglion_id < 3)
                        flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
                    else
                        flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    querySensoryData();
                    sleep(2);
                    refresh();
                    //Resetting the spring
                    flexray.controlparams.tag = 1;
                    flexray.commandframe0[0].OperationMode[0] = Initialise;
                    refresh();
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    flexray.updateMotorState();
                    querySensoryData();
                    flexray.controlparams.tag = 0;
                    flexray.commandframe0[0].OperationMode[0] = Initialise;
                    refresh();
                    flexray.updateCommandFrame();
                    flexray.exchangeData();
                    flexray.updateMotorState();
                    querySensoryData();
                    mvprintw(22 + s, 0, "Sensor Displacement Resetted Motor: %d Ganglion: %d ", motor_id, ganglion_id);
                    refresh();
                    s++;
                }
            }
        }
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }


    void positionControl() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        flexray.initPositionControl(ganglion_id, motor_id);
        printMessage(4, 0, setpointstring);
        mvchgat(4, 0, strlen(setpointstring), A_BOLD, 1, NULL);
        refresh();
        mvgetnstr(5, 0, inputstring, 30);
        pos = atof(inputstring);
        if (ganglion_id < 3)
            flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
        else
            flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
        flexray.updateCommandFrame();
        flexray.exchangeData();
        processing(runningstring, inputstring, quitstring);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void velocityControl() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        flexray.initVelocityControl(ganglion_id, motor_id, 100.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    0.0, -5.0, 5.0);
        printMessage(4, 0, setvelstring);
        mvchgat(4, 0, strlen(setvelstring), A_BOLD, 1, NULL);
        refresh();
        mvgetnstr(5, 0, inputstring, 30);
        pos = atof(inputstring);
        if (ganglion_id < 3)
            flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
        else
            flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
        flexray.updateCommandFrame();
        flexray.exchangeData();
        processing(runningstring, inputstring, quitstring);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void forceControl() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        flexray.initForceControl(ganglion_id, motor_id);
        printMessage(4, 0, setforcestring);
        mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
        refresh();
        mvgetnstr(5, 0, inputstring, 30);
        pos = atof(inputstring);
        if (ganglion_id < 3)
            flexray.commandframe0[ganglion_id].sp[motor_id] = pos;
        else
            flexray.commandframe1[ganglion_id - 3].sp[motor_id] = pos;
        flexray.updateCommandFrame();
        flexray.exchangeData();
        processing(runningstring, inputstring, quitstring);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void switchMotor() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        printMessage(4, 0, ganglionstring, GREEN);
        mvgetnstr(5, 0, inputstring, 30);
        uint ganlionrequest = atoi(inputstring);
        if (ganlionrequest < 6)
            ganglion_id = ganlionrequest;
        else {
            print(4, 0, cols, " ");
            print(5, 0, cols, " ");
            printMessage(5, 0, invalidstring, RED);
            return;
        }
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        printMessage(4, 0, motorstring, GREEN);
        mvgetnstr(5, 0, inputstring, 30);
        uint motorrequest = atoi(inputstring);
        if (motorrequest < 4)
            motor_id = motorrequest;
        else {
            print(4, 0, cols, " ");
            print(5, 0, cols, " ");
            printMessage(5, 0, invalidstring, RED);
            return;
        }
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void measureConnection() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        double averageTime = flexray.measureConnectionTime();
        printMessage(4, 0, averageconnectionspeedstring);
        char str[20];
        sprintf(str, "%f seconds", averageTime);
        printMessage(4, strlen(averageconnectionspeedstring), str, CYAN);
        printMessage(5, 0, logfilestring, CYAN);
        usleep(5000000);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void recordTrajectories() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        printMessage(4, 0, filenamestring);
        mvgetnstr(4, strlen(filenamestring), inputstring, 30);
        std::string name(inputstring);
        print(4, 0, cols, " ");
        printMessage(4, 0, samplingtimestring, CYAN);
        mvgetnstr(4, strlen(samplingtimestring), inputstring, 30);
        float samplingTime = atof(inputstring);
        printMessage(5, 0, recordtimestring, CYAN);
        mvgetnstr(5, strlen(recordtimestring), inputstring, 30);
        double recordTime = atof(inputstring);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        printMessage(4, 0, recordingstring, RED);
        std::vector<std::vector<float>> trajectories;
        std::vector<int> idList = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
        std::vector<int> controlmode(16, 1);
        float averageSamplingTime = flexray.recordTrajectories(samplingTime, recordTime, trajectories, idList,
                                                               controlmode, name);
        print(4, 0, cols, " ");
        printMessage(4, 0, donestring, GREEN);
        char averagetimestring[50];
        sprintf(averagetimestring, "average %s%f", samplingtimestring, averageSamplingTime);
        printMessage(4, strlen(donestring), averagetimestring, CYAN);
        usleep(500000);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
    }

    void setAllToForce() {
        timeout(-1);
        echo();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        printMessage(4, 0, setforcestring);
        mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
        refresh();
        mvgetnstr(5, 0, inputstring, 30);
        pos = atof(inputstring);
        flexray.initForceControl();
        for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
            for (uint motor = 0; motor < 4; motor++) {
                if (ganglion_id < 3)
                    flexray.commandframe0[ganglion].sp[motor] = pos;
                else
                    flexray.commandframe1[ganglion - 3].sp[motor] = pos;
            }
        }
        flexray.updateCommandFrame();
        processing(runningstring, inputstring, quitstring);
        // set back to zero force
        flexray.initForceControl();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void remoteAndroidControl() {
        motor_status_pub = nh->advertise<common_utilities::MotorStatus>("/roboy/motor_status", 1);
        motor_cmd_pos_sub = nh->subscribe("/roboy/motor_cmd_pos", 100, &NCurses_flexray::processPositionCommand, this);
        motor_cmd_vel_sub = nh->subscribe("/roboy/motor_cmd_vel", 100, &NCurses_flexray::processVelocityCommand, this);
        motor_cmd_force_sub = nh->subscribe("/roboy/motor_cmd_force", 100, &NCurses_flexray::processForceCommand, this);
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        flexray.initForceControl();
        print(5, 0, cols, " ");
        printMessage(5, 0, remotecontrolactivestring, RED);
        timeout(10);
        char cmd;
        do {
            common_utilities::MotorStatus msg;
            for (int ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
                for (int motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
                    msg.jointPos.push_back(flexray.GanglionData[ganglion].muscleState[motor].jointPos);
                    msg.actuatorPos.push_back(flexray.GanglionData[ganglion].muscleState[motor].actuatorPos *
                                              flexray.controlparams.radPerEncoderCount);
                    msg.actuatorVel.push_back(flexray.GanglionData[ganglion].muscleState[motor].actuatorVel *
                                              flexray.controlparams.radPerEncoderCount);
                    msg.actuatorCurrent.push_back(flexray.GanglionData[ganglion].muscleState[motor].actuatorCurrent);
                    msg.tendonDisplacement.push_back(
                            flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement);
                }
            }
            motor_status_pub.publish(msg);

            mvchgat(5, 0, strlen(remotecontrolactivestring), A_BLINK, 2, NULL);
            cmd = mvgetch(5, 22);
        } while (cmd != 'q');
        motor_status_pub.shutdown();
        motor_cmd_pos_sub.shutdown();
        motor_cmd_vel_sub.shutdown();
        motor_cmd_force_sub.shutdown();
        // set back to zero force
        flexray.initForceControl();
        print(4, 0, cols, " ");
        print(5, 0, cols, " ");
        noecho();
    }

    void publishMotorInfo(){
        flexray.initForceControl();
        char cmd;
        vector<ros::Publisher> displacement_pub(NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION);
        int i=0;
        for (int ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
            for (int motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
                char topic[100];
                sprintf(topic, "/roboy/ganglion%d/motor%d/displacement", ganglion, motor);
                displacement_pub[i++] = nh->advertise<std_msgs::Float32>(topic, 100);
                if (ganglion_id < 3)
                    flexray.commandframe0[ganglion].sp[motor] = 6.0f;
                else
                    flexray.commandframe1[ganglion - 3].sp[motor] = 6.0f;
            }
        }
        flexray.updateCommandFrame();
        do {
            flexray.exchangeData();
            i=0;
            for (int ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
                for (int motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
                    std_msgs::Float32 msg;
                    msg.data = flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement / 32768.0f;
                    displacement_pub[i++].publish(msg);
                }
            }
            mvchgat(5, 0, strlen(publishingmotorstring), A_BLINK, 2, NULL);
            cmd = mvgetch(5, strlen(publishingmotorstring));
        } while (cmd != 'q');
        i=0;
        for (int ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
            for (int motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
                displacement_pub[i++].shutdown();
            }
        }
    }

    void processPositionCommand(const common_utilities::MotorCommand::ConstPtr &msg) {
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION > flexray.numberOfGanglionsConnected) {
            printMessage(4, 0, errormessage, RED);
            return;
        }
        uint ganglion = msg->id/NUMBER_OF_JOINTS_PER_GANGLION;
        uint motor = msg->id%NUMBER_OF_JOINTS_PER_GANGLION;
        flexray.initPositionControl(motor, ganglion);
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION < 3)
            flexray.commandframe0[ganglion].sp[motor] = msg->setpoint;
        else
            flexray.commandframe1[ganglion-3].sp[motor] = msg->setpoint;
        printMessage(4, 0, receivedupdatestring, GREEN);
        flexray.updateCommandFrame();
        flexray.exchangeData();
    }

    void processVelocityCommand(const common_utilities::MotorCommand::ConstPtr &msg) {
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION > flexray.numberOfGanglionsConnected) {
            printMessage(4, 0, errormessage, RED);
            return;
        }
        uint ganglion = msg->id/NUMBER_OF_JOINTS_PER_GANGLION;
        uint motor = msg->id%NUMBER_OF_JOINTS_PER_GANGLION;
        flexray.initVelocityControl(motor, ganglion);
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION < 3)
            flexray.commandframe0[ganglion].sp[motor] = msg->setpoint;
        else
            flexray.commandframe1[ganglion-3].sp[motor] = msg->setpoint;
        printMessage(4, 0, receivedupdatestring, GREEN);
        flexray.updateCommandFrame();
        flexray.exchangeData();
    }

    void processForceCommand(const common_utilities::MotorCommand::ConstPtr &msg) {
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION > flexray.numberOfGanglionsConnected) {
            printMessage(4, 0, errormessage, RED);
            return;
        }
        uint ganglion = msg->id/NUMBER_OF_JOINTS_PER_GANGLION;
        uint motor = msg->id%NUMBER_OF_JOINTS_PER_GANGLION;
        flexray.initForceControl(motor, ganglion);
        if(msg->id/NUMBER_OF_JOINTS_PER_GANGLION < 3)
            flexray.commandframe0[ganglion].sp[motor] = msg->setpoint;
        else
            flexray.commandframe1[ganglion-3].sp[motor] = msg->setpoint;
        printMessage(4, 0, receivedupdatestring, GREEN);
        flexray.updateCommandFrame();
        flexray.exchangeData();
    }

private:
    FlexRayHardwareInterface flexray;
    uint rows, cols;
    float pos;
    uint ganglion_id = 0;
    uint motor_id = 0;
    char inputstring[30];
    MotorData motor;
    ros::NodeHandlePtr nh;
    ros::Publisher motor_status_pub;
    ros::Subscriber motor_cmd_pos_sub, motor_cmd_vel_sub, motor_cmd_force_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
};
