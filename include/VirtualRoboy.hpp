#pragma once
#include "CommunicationData.h"
#include "timer.hpp"
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <math.h>
#include <iostream>

using namespace std;

class VirtualPIDController
{
public:
    /** Constructor Dtart of control thread
     * @param controlparams - pointer to control parameters
     */
    VirtualPIDController(sint8 *cm, sint8 *om, float32 *setpoint, control_Parameters_t *controlparams,
						 muscleState_t *muscleState, float *controllerOutput):
			m_controlparams(controlparams), m_muscleState(muscleState), controlMode(cm),
			OperationMode(om), sp(setpoint), control(controllerOutput){

		// point to respective parameters
		tag = &controlparams->tag;
		outputNegMax = &controlparams->outputNegMax;
		outputPosMax = &controlparams->outputPosMax;
		spPosMax = &controlparams->spPosMax;
		spNegMax = &controlparams->spNegMax;
		radPerEncoderCount = &controlparams->radPerEncoderCount;
		polyPar[0] = &controlparams->polyPar[0];
		polyPar[1] = &controlparams->polyPar[1];
		polyPar[2] = &controlparams->polyPar[2];
		polyPar[3] = &controlparams->polyPar[3];
		torqueConstant = &controlparams->torqueConstant;
		params = &controlparams->params;

		// start the control thread
		control_thread = new std::thread(&VirtualPIDController::control_loop, this);
	};

	/** Destructor*/
	~VirtualPIDController() {
		if (control_thread != nullptr) {
			isEnabled = false;
			control_thread->join();
		}
	}

	/** control loop started in extra thread */
	void control_loop(){
		timer.start();
		double dt,elapsedTime=0.0;

		while(isEnabled){
			dt = elapsedTime;

			*control = outputCalc();

			elapsedTime = timer.elapsedTimeMicroSeconds();
			dt = elapsedTime - dt;
			// if faster than timePeriod sleep for difference
			if(dt < *timePeriod) {
				usleep(*timePeriod - dt);
				elapsedTime = timer.elapsedTimeMicroSeconds();
			}
		}
	}

    /** Calculate the control output.
    * 	@return The control output.
    */
    float outputCalc(){
		float pterm, dterm, result, err, ffterm;

		if(*sp >= *spNegMax && *sp <= *spPosMax) {
			switch (*controlMode) {
				case Position:
					err = m_muscleState->actuatorPos - *sp;
					break;
				case Velocity:
					err = m_muscleState->actuatorVel - *sp;
					break;
				case Torque:
					err = ((*polyPar[0]) +
						   (*polyPar[1]) * m_muscleState->tendonDisplacement +
						   (*polyPar[2]) * powf(m_muscleState->tendonDisplacement, 2.0f) +
						   (*polyPar[3]) * powf(m_muscleState->tendonDisplacement, 3.0f)) - *sp;
					break;
			}
			if ((err > params->pidParameters.deadBand) || (err < -1 * params->pidParameters.deadBand)) {
				pterm = params->pidParameters.pgain * err;
				if ((pterm < *outputPosMax) || (pterm > *outputNegMax)) //if the proportional term is not maxed
				{
					integral += (params->pidParameters.igain * err * (*timePeriod)); //add to the integral
					if (integral > params->pidParameters.IntegralPosMax)
						integral = params->pidParameters.IntegralPosMax;
					else if (integral < params->pidParameters.IntegralNegMax)
						integral = params->pidParameters.IntegralNegMax;
				}

				dterm = ((err - lastError) / (*timePeriod)) * params->pidParameters.dgain;

				ffterm = params->pidParameters.forwardGain * (*sp);
				result = ffterm + pterm + integral + dterm;
				if (result < *outputNegMax)
					result = *outputNegMax;
				else if (result > *outputPosMax)
					result = *outputPosMax;
			}
			else
				result = integral;
			lastError = err;
		}else {
			result = 0;
		}
		return result;
	}

private:
	uint32 *tag;/*!<Tag to indicate data type when passing the union*/
	sint32 *outputPosMax; /*!< maximum control output in the positive direction in counts, max 4000*/
	sint32 *outputNegMax; /*!< maximum control output in the negative direction in counts, max -4000*/
	float32 *spPosMax;/*<!Positive limit for the set point.*/
	float32 *spNegMax;/*<!Negative limit for the set point.*/
	float32 *timePeriod;/*!<Time period of each control iteration in microseconds.*/
	float32 *radPerEncoderCount; /*!output shaft rotation (in rad) per encoder count */
	float32 *polyPar[4]; /*! polynomial fit from displacement (d)  to tendon force (f)
			 f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3/ //*/
	float32 *torqueConstant; /*!motor torque constant in Nm/A */
	parameters_t *params;
	sint8 *controlMode;
	sint8 *OperationMode;
	float32 *sp;
	float32 *control;
	float32 integral;
	float32 lastError;
	control_Parameters_t *m_controlparams;
	muscleState_t *m_muscleState;
	bool isEnabled = false;
	thread *control_thread = nullptr;
	Timer timer;
};

class VirtualGanglion{
public:
	VirtualGanglion(comsCommandFrame *commandFrame, control_Parameters_t *controlparams, ganglionData_t *GanglionData, float* controllerOutput):
			m_commandFrame(commandFrame), m_controlparams(controlparams), m_GanglionData(GanglionData){
		m_controller.resize(NUMBER_OF_JOINTS_PER_GANGLION);
		// each controller
		for(uint i=0; i<NUMBER_OF_JOINTS_PER_GANGLION; i++){
			m_controller[i] = new VirtualPIDController(&commandFrame->ControlMode[i],&commandFrame->OperationMode[i],&commandFrame->sp[i],
													   controlparams, &GanglionData->muscleState[i], &controllerOutput[i]);
		}
	}
	~VirtualGanglion(){
		for(uint i=0; i<NUMBER_OF_JOINTS_PER_GANGLION; i++){
			delete m_controller[i];
		}
	}
private:
	vector<VirtualPIDController*> m_controller;
	comsCommandFrame* m_commandFrame;
	control_Parameters_t* m_controlparams;
	ganglionData_t *m_GanglionData;
};

class VirtualRoboy{
public:
	VirtualRoboy(ganglionData_t *GanglionData, comsCommandFrame *commandframe0,
				 comsCommandFrame *commandframe1, control_Parameters_t *controlparams){
		m_ganglia.resize(NUMBER_OF_GANGLIONS);
		// each ganglion gets a reference to its commandframe and its ganglion data frame
		for(uint i=0; i<NUMBER_OF_GANGLIONS; i++){
			if(i<3) {
				m_ganglia[i] = new VirtualGanglion(commandframe0, controlparams, GanglionData, &controllerOutput[i*NUMBER_OF_JOINTS_PER_GANGLION]);
				++commandframe0;
			}else {
				m_ganglia[i] = new VirtualGanglion(commandframe1, controlparams, GanglionData, &controllerOutput[i*NUMBER_OF_JOINTS_PER_GANGLION]);
				++commandframe1;
			}
			++GanglionData;
		}
	}
	~VirtualRoboy(){
		for(uint i=0; i<NUMBER_OF_GANGLIONS; i++){
			delete m_ganglia[i];
		}
	}
	float controllerOutput[NUMBER_OF_GANGLIONS*NUMBER_OF_JOINTS_PER_GANGLION];
private:
	mutex mux; // TODO: check thread safety, possibly use mutex while changing values in GanglionData/commandframes/controlparams
	vector<VirtualGanglion*> m_ganglia;
};