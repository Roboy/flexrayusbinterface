#pragma once
#include "CommunicationData.h"
#include <stdlib.h>
#include <string>
#include <vector>

class VirtualPIDController
{
	uint tag;
	bool isEnabled = false;
    float32 integral;/*!<Integral of the error*/
    float32 pgain;/*!<Gain of the proportional component*/
    float32 igain;/*!<Gain of the integral component*/
    float32 dgain;/*!<Gain of the differential component*/
    float32 forwardGain; /*!<Gain of  the feed-forward term*/
    float32 deadBand;/*!<Optional deadband threshold for the control response*/
    float32 lastError;/*!<Error in previous time-step, used to calculate the differential component*/
    float32 IntegralPosMax;/*<!Positive saturation limit for the integral term*/
    float32 IntegralNegMax;/*<!Negative saturation limit for the integral term*/
    float32 outputPosMax;
    float32 outputNegMax;
    float32 spPosMax;
    float32 spNegMax;
	float32 radPerEncoderCount;
	float32 polyPar[4];
	float32 torqueConstant;
	float32 setPoint;
    uint32 timePeriod;

    public:

    /** Constructor to instantiate a PID controller with the variables stored in a
     * 	control_Parameters union.*/
    VirtualPIDController(const control_Parameters_t&);

    /** Constructor to instantiate an uninitialised PID controller.*/
    VirtualPIDController(){};

    /** Set the integral term to a given value.
    *	@param new_integ The value to which the integral term will be set.
    */
    void pid_setinteg(float new_integ);

    /** Get the gain of the P element of the controller.
    *	@return The P gain of the controller.
    */
    float get_pgain(){return pgain;}

    /** Get the gain of the I element of the controller.
    *	@return The I gain of the controller.
    */
    float get_igain(){return igain;}

    /** Get the gain of the D element of the controller.
    *	@return The D gain of the controller.
    */
    float get_dgain(){return dgain;}

    /** Set the P gain of the controller.
    *	@param gain The gain value to be set.
    *	@return 0 on success, 1 if an illegal value is requested.
    */
    int set_pgain(float gain);

    /** Set the I gain of the controller.
    *	@param gain The gain value to be set.
    *	@return 0 on success, 1 if an illegal value is requested.
    */
    int set_igain(float gain);

    /** Set the D gain of the controller.
    *	@param gain The gain value to be set.
    *	@return 0 on success, 1 if an illegal value is requested.
    */
    int set_dgain(float gain);

    /** Set the positive saturation limit for the integral term in the controller.
    *	@param limit The limit value to be set.
    *	@return 0 on success, 1 if an illegal value is requested.
    */
    int set_IntegralPosMax(float limit);

    /** Set the negative saturation limit for the integral term in the controller.
    *	@param limit The limit value to be set.
    *	@return 0 on success, 1 if an illegal value is requested.
    */
    int set_IntegralNegMax(float limit);

    /** Set all the parameters in the controller.
    *	@param parameters A structure that contains all the parameter data.
    *	@return 0 on success.
    */
    int setParams(const control_Parameters_t& parameters);

    /** Get all the parameters in the controller.
    *	@return parameters structure that will contain all the parameter data.
    */
    control_Parameters_t getParams();

    /** Calculate the control output.
    * 	@param pv The current process value of the system.
    * 	@return The control output.
    */
    float outputCalc(float pv);

    virtual void setisEnabled(bool enable);
};

class VirtualGanglion{
public:
	VirtualGanglion(){
		controller.resize(NUMBER_OF_JOINTS_PER_GANGLION);
	}
private:
	std::vector<VirtualPIDController> controller;
};

class VirtualRoboy{
public:
	VirtualRoboy(){
		ganglia.resize(NUMBER_OF_GANGLIONS);
	}

private:
	std::vector<VirtualGanglion> ganglia;
};