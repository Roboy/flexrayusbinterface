#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"

FlexRayHardwareInterface::FlexRayHardwareInterface() {
    motorState.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION, 1);
    motorControllerType.resize(
            NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION, 0);
#ifdef HARDWARE
    ROS_INFO("Trying to connect to FlexRay");
    while (!connect()) {
        ROS_INFO("retrying...");
        usleep(1000000); // sleep for a second
    }
#else
    ROS_DEBUG("No Hardware mode enabled");
    activeGanglionsMask = 0b111111;
    numberOfGanglionsConnected = 6;
#endif
    initializeMotors();
};

FlexRayHardwareInterface::~FlexRayHardwareInterface() {
}

bool FlexRayHardwareInterface::connect() {
    if (CheckDeviceConnected(&m_numberOfConnectedDevices) == true) {
        if (GetDeviceInfo(&m_numberOfConnectedDevices) == true) {
            if (OpenPortAndConfigureMPSSE(&m_ftHandle, USBINSIZE, USBOUTSIZE) ==
                true) {
                bool mpssetested = false;
                uint tries = 0;
                do {
                    mpssetested = TestMPSSE(&m_ftHandle);
                    if (mpssetested) {
                        bool spiconfigured = false;
                        do {
                            spiconfigured = ConfigureSPI(&m_ftHandle, m_clockDivisor);
                        } while (spiconfigured == false);
                        ROS_INFO("connection OK");
                        return true;
                    }
                    ROS_INFO("Testing MPSSE failed, retrying!");
                    tries++;
                } while (mpssetested == false && tries < 3); // TestMPSSE
            } // OpenPortAndConfigureMPSSE
            else {
                ROS_ERROR("open port failed\n--Perhaps the kernel automatically loaded "
                                  "another driver for the FTDI USB device, from command line "
                                  "try: \nsudo rmmod ftdi_sio \n sudo rmmod usbserial\n or "
                                  "maybe run with sudo");
            }
        } // GetDeviceInfo
        else {
            ROS_ERROR("device info failed");
        }
    } // CheckDeviceConnected
    else {
        ROS_ERROR("device not connected");
    }
    return false;
};

void FlexRayHardwareInterface::initializeMotors() {
    controlparams.tag = 0;              // sint32
    controlparams.outputPosMax = 1000;  // sint32
    controlparams.outputNegMax = -1000; // sint32
    controlparams.timePeriod = 100; // float32      //in us set time period to avoid error case

    controlparams.radPerEncoderCount =
            2 * 3.14159265359 / (2000.0 * 53.0);          // float32
    controlparams.params.pidParameters.lastError = 0; // float32

    updateCommandFrame();
    initForceControl();

#ifdef HARDWARE
    checkNumberOfConnectedGanglions();
    updateMotorState();
#else
    ROS_DEBUG("NO HARDWARE MODE");
    numberOfGanglionsConnected = 6;
#endif
    ROS_INFO("%d ganglions are connected via flexray, activeGanglionsMask %c",
             numberOfGanglionsConnected, activeGanglionsMask);
};

void FlexRayHardwareInterface::relaxSpring(uint ganglion_id, uint motor_id) {

    // make a backup of control modes so after recording they can be restored
    std::vector<int8_t> motorControllerType_backup = motorControllerType;
    std::vector<float> setPoints_backup(NUMBER_OF_GANGLIONS *
                                        NUMBER_OF_JOINTS_PER_GANGLION);
    uint i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                setPoints_backup[i] = commandframe0[ganglion].sp[motor];
            else
                setPoints_backup[i] = commandframe1[ganglion].sp[motor];
            i++;
        }
    }

    // set all motors into velocity mode
    initVelocityControl();
    float vel = 3;
    if (ganglion_id < 3)
        commandframe0[ganglion_id].sp[motor_id] = vel;
    else
        commandframe1[ganglion_id - 3].sp[motor_id] = vel;
    updateCommandFrame();
    exchangeData();
    // pull on our motor until it is completely unrolled
    float tendonDisplacement_old = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement/32768.0f;
    float tendonDisplacement_new = tendonDisplacement_old + 1;
    bool unrolled = false;
    while(!unrolled){
        float diff = tendonDisplacement_new-tendonDisplacement_old;
        if(diff<0){
            ROS_INFO_THROTTLE(1,"tendon displacement smaller than zero %f", diff);
        }else if(fabs(diff)<0.00001){
            ROS_INFO("spring relaxed %f", diff);
            break;
        }else{
            // reverse direction
            ROS_INFO("reversing direction %f", diff);
            if (ganglion_id < 3)
                commandframe0[ganglion_id].sp[motor_id] *= -1;
            else
                commandframe1[ganglion_id - 3].sp[motor_id] *= -1;
            updateCommandFrame();
        }
        exchangeData();
        tendonDisplacement_old = tendonDisplacement_new;
        tendonDisplacement_new = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement/32768.0f;
    }

//    float tendonDisplacement_t[3];
//    tendonDisplacement_t[0] = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement / 32768.0f; // tendon displacement zero
//    // set the motor for resetting the spring to velocity mode
//    initVelocityControl(ganglion_id, motor_id);
//
//    ROS_INFO("Displacement_t0 %.5f ", tendonDisplacement_t[0]);
//
//    if (ganglion_id < 3)
//        commandframe0[ganglion_id].sp[motor_id] = 1;
//    else
//        commandframe1[ganglion_id - 3].sp[motor_id] = 1;
//    updateCommandFrame();
//    exchangeData();
//    usleep(500000);
//    exchangeData();
//    tendonDisplacement_t[1] = GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement /32768.0f;
//    ROS_INFO("Displacement_t1 %.5f ", tendonDisplacement_t[1]);

//    float vel;
//    if (tendonDisplacement_t[1] > tendonDisplacement_t[0])
//        vel = -3;
//    else if (tendonDisplacement_t[1] < tendonDisplacement_t[0])
//        vel = 3;
//    else
//        vel = 0;
//
//
//
//    do {
//        if (ganglion_id < 3)
//            commandframe0[ganglion_id].sp[motor_id] = vel;
//        else
//            commandframe1[ganglion_id - 3].sp[motor_id] = vel;
//        updateCommandFrame();
//        exchangeData();
//        usleep(300000);
//        updateCommandFrame();
//        exchangeData();
//        tendonDisplacement_t2[t + 1] =
//                GanglionData[ganglion_id].muscleState[motor_id].tendonDisplacement /
//                32768.0f;
//        t++;
//        ROS_INFO("Displacement_t2 %d %.5f ", t, tendonDisplacement_t2[t]);
//
//        if (tendonDisplacement_t2[t] > tendonDisplacement_t2[t - 1])
//            p++;
//
//        else if (tendonDisplacement_t2[t] == tendonDisplacement_t2[t - 1]) {
//
//            if (count > 0 && tendonDisplacement_t2[t] == aux)
//                count++;
//            else if (count == 0) {
//                aux = tendonDisplacement_t2[t];
//                count++;
//            } else
//                count = 0;
//        }
//
//    } while (count != 4 && p != 3);


    // restore controller types
    uint m = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            switch (motorControllerType_backup[m]) {
                case Position:
                    initPositionControl(ganglion, motor);
                    break;
                case Velocity:
                    initVelocityControl(ganglion, motor);
                    break;
                case Force:
                    initForceControl(ganglion, motor);
                    break;
            }
            m++;
        }
    }
    // restore setpoints
    i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = setPoints_backup[i];
            else
                commandframe1[ganglion - 3].sp[motor] = setPoints_backup[i];
            i++;
        }
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initPositionControl(
        float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
        float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
        float spPosMax) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    // initialize PID controller in motordriver boards
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].ControlMode[motor] = Position;
            commandframe0[i].OperationMode[motor] = Initialise;
            commandframe1[i].ControlMode[motor] = Position;
            commandframe1[i].OperationMode[motor] = Initialise;
            motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Position;
            motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) *
                                NUMBER_OF_JOINTS_PER_GANGLION +
                                motor] = Position;
        }
    }
    updateCommandFrame();
    exchangeData();
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].OperationMode[motor] = Run;
            float pos = GanglionData[i].muscleState[motor].actuatorPos;
            commandframe0[i].sp[motor] = pos * controlparams.radPerEncoderCount;
            commandframe1[i].OperationMode[motor] = Run;
            pos = GanglionData[i + 3].muscleState[motor].actuatorPos;
            commandframe1[i].sp[motor] = pos * controlparams.radPerEncoderCount;
        }
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initPositionControl(
        uint ganglion, uint motor, float Pgain, float IGain, float Dgain,
        float forwardGain, float deadBand, float integral, float IntegralPosMin,
        float IntegralPosMax, float spPosMin, float spPosMax) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    // initialize PID controller in motordriver boards
    if (ganglion < 3) {
        commandframe0[ganglion].ControlMode[motor] = Position;
        commandframe0[ganglion].OperationMode[motor] = Initialise;
    } else {
        commandframe1[ganglion - 3].ControlMode[motor] = Position;
        commandframe1[ganglion - 3].OperationMode[motor] = Initialise;
    }
    motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] =
            Position;
    updateCommandFrame();
    exchangeData();
    float pos = GanglionData[ganglion].muscleState[motor].actuatorPos *
                controlparams.radPerEncoderCount;
    if (ganglion < 3) {
        commandframe0[ganglion].OperationMode[motor] = Run;
        commandframe0[ganglion].sp[motor] = pos;
    } else {
        commandframe1[ganglion - 3].OperationMode[motor] = Run;
        commandframe1[ganglion - 3].sp[motor] = pos;
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(
        float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
        float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
        float spPosMax) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    // initialize PID controller in motordriver boards
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].ControlMode[motor] = Velocity;
            commandframe0[i].OperationMode[motor] = Initialise;
            commandframe1[i].ControlMode[motor] = Velocity;
            commandframe1[i].OperationMode[motor] = Initialise;
            motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Velocity;
            motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) *
                                NUMBER_OF_JOINTS_PER_GANGLION +
                                motor] = Velocity;
        }
    }
    updateCommandFrame();
    exchangeData();
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].OperationMode[motor] = Run;
            commandframe0[i].sp[motor] = 0;
            commandframe1[i].OperationMode[motor] = Run;
            commandframe1[i].sp[motor] = 0;
        }
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initVelocityControl(
        uint ganglion, uint motor, float Pgain, float IGain, float Dgain,
        float forwardGain, float deadBand, float integral, float IntegralPosMin,
        float IntegralPosMax, float spPosMin, float spPosMax) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    // initialize PID controller in motordriver boards
    if (ganglion < 3) {
        commandframe0[ganglion].ControlMode[motor] = Velocity;
        commandframe0[ganglion].OperationMode[motor] = Initialise;
    } else {
        commandframe1[ganglion].ControlMode[motor] = Velocity;
        commandframe1[ganglion].OperationMode[motor] = Initialise;
    }
    motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] =
            Velocity;
    updateCommandFrame();
    exchangeData();
    if (ganglion < 3) {
        commandframe0[ganglion].OperationMode[motor] = Run;
        commandframe0[ganglion].sp[motor] = 0;
    } else {
        commandframe1[ganglion - 3].OperationMode[motor] = Run;
        commandframe1[ganglion - 3].sp[motor] = 0;
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initForceControl(
        float Pgain, float IGain, float Dgain, float forwardGain, float deadBand,
        float integral, float IntegralPosMin, float IntegralPosMax, float spPosMin,
        float spPosMax, float torqueConstant, char springType) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    float polyPar[4];

    switch (springType) {
        // f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4
        case SoftSpring: // D311-spring 0.621249
            polyPar[0] = 0;
            polyPar[1] = 0.237536;
            polyPar[2] = -0.000032;
            polyPar[3] = 0;
            break;
        case MiddleSpring:
            polyPar[0] = 1.604382;
            polyPar[1] = 0.508932;
            polyPar[2] = -0.000117;
            polyPar[3] = 0;
            break;
        case HardSpring:
            polyPar[0] = 1.186483;
            polyPar[1] = 0.938377;
            polyPar[2] = -0.000274;
            polyPar[3] = 0.000001;
            break;
    }

    controlparams.polyPar[0] = polyPar[0]; // float32
    controlparams.polyPar[1] = polyPar[1];
    controlparams.polyPar[2] = polyPar[2];
    controlparams.polyPar[3] = polyPar[3];
    controlparams.torqueConstant = torqueConstant; // float32

    // initialize PID controller in motordriver boards
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].ControlMode[motor] = Force;
            commandframe0[i].OperationMode[motor] = Initialise;
            commandframe1[i].ControlMode[motor] = Force;
            commandframe1[i].OperationMode[motor] = Initialise;
            motorControllerType[i * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;
            motorControllerType[(i + NUMBER_OF_GANGLIONS / 2) *
                                NUMBER_OF_JOINTS_PER_GANGLION +
                                motor] = Force;
        }
    }
    updateCommandFrame();
    exchangeData();
    for (uint i = 0; i < NUMBER_OF_GANGLIONS / 2; i++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            commandframe0[i].OperationMode[motor] = Run;
            commandframe0[i].sp[motor] = 0;
            commandframe1[i].OperationMode[motor] = Run;
            commandframe1[i].sp[motor] = 0;
        }
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::initForceControl(
        uint ganglion, uint motor, float Pgain, float IGain, float Dgain,
        float forwardGain, float deadBand, float integral, float IntegralPosMin,
        float IntegralPosMax, float spPosMin, float spPosMax, float torqueConstant,
        char springType) {
    controlparams.spPosMax = spPosMax; // float32
    controlparams.spNegMax = spPosMin; // float32

    controlparams.params.pidParameters.integral = integral;             // float32
    controlparams.params.pidParameters.pgain = Pgain;                   // float32
    controlparams.params.pidParameters.igain = IGain;                   // float32
    controlparams.params.pidParameters.dgain = Dgain;                   // float32
    controlparams.params.pidParameters.forwardGain = forwardGain;       // float32
    controlparams.params.pidParameters.deadBand = deadBand;             // float32
    controlparams.params.pidParameters.IntegralPosMax = IntegralPosMax; // float32
    controlparams.params.pidParameters.IntegralNegMax = IntegralPosMin; // float32

    float polyPar[4];

    switch (springType) {
        // f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3+ +polyPar[4]*d^4
        case SoftSpring: // D311-spring
            polyPar[0] = 0;
            polyPar[1] = 0.237536;
            polyPar[2] = -0.000032;
            polyPar[3] = 0;
            break;
        case MiddleSpring:
            polyPar[0] = 1.604382;
            polyPar[1] = 0.508932;
            polyPar[2] = -0.000117;
            polyPar[3] = 0;
            break;
        case HardSpring:
            polyPar[0] = 1.186483;
            polyPar[1] = 0.938377;
            polyPar[2] = -0.000274;
            polyPar[3] = 0.000001;
            break;
    }

    controlparams.polyPar[0] = polyPar[0]; // float32
    controlparams.polyPar[1] = polyPar[1];
    controlparams.polyPar[2] = polyPar[2];
    controlparams.polyPar[3] = polyPar[3];
    controlparams.torqueConstant = torqueConstant; // float32

    // initialize PID controller in motordriver boards
    if (ganglion < 3) {
        commandframe0[ganglion].ControlMode[motor] = Force;
        commandframe0[ganglion].OperationMode[motor] = Initialise;
    } else {
        commandframe1[ganglion - 3].ControlMode[motor] = Force;
        commandframe1[ganglion - 3].OperationMode[motor] = Initialise;
    }
    motorControllerType[ganglion * NUMBER_OF_JOINTS_PER_GANGLION + motor] = Force;

    updateCommandFrame();
    exchangeData();
    if (ganglion < 3) {
        commandframe0[ganglion].OperationMode[motor] = Run;
        commandframe0[ganglion].sp[motor] = 0;
    } else {
        commandframe1[ganglion - 3].OperationMode[motor] = Run;
        commandframe1[ganglion - 3].sp[motor] = 0;
    }
    updateCommandFrame();
    exchangeData();
}

void FlexRayHardwareInterface::exchangeData() {
#ifdef HARDWARE
    ftStatus = SPI_WriteBuffer(m_ftHandle, &dataset[0], DATASETSIZE); // send data
    if (ftStatus != FT_OK) {
        ROS_ERROR_STREAM(
                "Failed to Send a byte through SPI, Error Code: " << ftStatus);
        // FT_SetBitMode(ftHandle, 0x0, 0x00);           // Reset the port to
        // disable MPSSE
        // FT_Close(ftHandle);                   // Close the USB port
    } else {
        DWORD dwNumInputBuffer = 0;
        DWORD dwNumBytesRead;

        // WAIT FOR DATA TO ARRIVE
        ROS_DEBUG("waiting for data");
        while (dwNumInputBuffer != DATASETSIZE * 2) {
            ftStatus = FT_GetQueueStatus(
                    m_ftHandle, &dwNumInputBuffer); // get the number of bytes in the
            // device receive buffer
        }
        if (ftStatus != FT_OK) {
            getErrorMessage(ftStatus, errorMessage);
            ROS_ERROR_STREAM("exchange data failed with error " << errorMessage);
        }
        ROS_DEBUG("reading data");
        FT_Read(m_ftHandle, &InputBuffer[0], dwNumInputBuffer,
                &dwNumBytesRead); // read bytes into word locations

        // now make a copy of relevant signal
        memcpy(GanglionData, InputBuffer, sizeof(GanglionData)); // ganglion data

        activeGanglionsMask =
                InputBuffer[sizeof(GanglionData) >>
                                                 1]; // active ganglions, generated from usbFlexRay interface
    }
#else
    ROS_DEBUG("NO HARDWARE");
#endif
    updateMotorState(); // ogni volta che scrivo e leggo faccio un update del
    // motor state (cambia da 0 a 1, da 0 a 1)
}

void FlexRayHardwareInterface::updateCommandFrame() {
    memcpy((void *) &dataset[0], commandframe0,
           sizeof(comsCommandFrame) * GANGLIONS_PER_CONTROL_FRAME);
    memcpy((void *) &dataset[sizeof(comsCommandFrame) *
                             GANGLIONS_PER_CONTROL_FRAME / 2],
           commandframe1, sizeof(comsCommandFrame) * GANGLIONS_PER_CONTROL_FRAME);
    memcpy((void *) &dataset[72], &controlparams, sizeof(control_Parameters_t));
}

// Nel secondo menmcp /2
// nel terzo ḿemcp e'72

uint32_t FlexRayHardwareInterface::NumberOfSetBits(uint32_t i) {
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

uint32_t FlexRayHardwareInterface::checkNumberOfConnectedGanglions() {
    exchangeData();
#ifdef HARDWARE
    numberOfGanglionsConnected = NumberOfSetBits(activeGanglionsMask);
#endif
    return numberOfGanglionsConnected;
}

void FlexRayHardwareInterface::updateMotorState() {
    uint m = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            int8_t status;
            if (GanglionData[ganglion].muscleState[motor].actuatorCurrent != 0) {
                status = 1;
            } else {
                status = 0;
            }
            motorState[m] = status;
            m++;
        }
    }
}

double FlexRayHardwareInterface::measureConnectionTime() {
    std::ofstream file;
    file.open("measureConnectionTime.log");
    timer.start();
    for (uint i = 0; i < 1000; i++) {
        updateCommandFrame();
        exchangeData();
    }
    double elapsedTime = timer.stop() / 1000.0;
    file << "Measured runtime for 1000 updateCommandFrame() + exchangeData() "
            "calls\n";
    file << "average time: " << elapsedTime << " seconds" << std::endl;
    file.close();
    return elapsedTime;
}

float FlexRayHardwareInterface::recordTrajectories(
        float samplingTime, float recordTime,
        std::vector<std::vector<float>> &trajectories, std::vector<int> &idList,
        std::vector<int> &controlmode, std::string name) {
    // this will be filled with the trajectories
    trajectories.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);

    // make a backup of control modes so after recording they can be restored
    std::vector<int8_t> motorControllerType_backup = motorControllerType;
    std::vector<float> setPoints_backup(NUMBER_OF_GANGLIONS *
                                        NUMBER_OF_JOINTS_PER_GANGLION);
    uint i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                setPoints_backup[i] = commandframe0[ganglion].sp[motor];
            else
                setPoints_backup[i] = commandframe1[ganglion].sp[motor];
            i++;
        }
    }

    // force control is probably the most convenient mode for recording
    initForceControl();
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = 20.0f;
            else
                commandframe1[ganglion - 3].sp[motor] = 20.0f;
        }
    }
    updateCommandFrame();
    exchangeData();

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    while (elapsedTime < recordTime) {
        dt = elapsedTime;
        exchangeData();
        for (uint m = 0; m < idList.size(); m++) {
            if (motorState[idList[m]] == 1) { // only record if motor is available
                if (controlmode[m] == 1)
                    trajectories.at(idList[m]).push_back(
                            GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                    .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                    .actuatorPos *
                            controlparams.radPerEncoderCount);
                else if (controlmode[m] == 2)
                    trajectories.at(idList[m]).push_back(
                            GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                    .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                    .actuatorVel *
                            controlparams.radPerEncoderCount);
                else if (controlmode[m] == 3)
                    trajectories.at(idList[m]).push_back(
                            GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                    .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                    .tendonDisplacement);
            }
        }
        sample++;
        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime) {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    }
    // done recording
    std::ofstream outfile;
    if (name.empty()) {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char str[200];
        sprintf(str, "/home/letrend/workspace/roboy-ros-control/src/"
                        "flexrayusbinterface/recording_%s.log",
                asctime(timeinfo));
        name = str;
    }

    outfile.open(name);
    if (outfile.is_open()) {
        outfile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>"
                << std::endl;
        outfile << "roboybehavior name=" << name << " behaviorid=\"200\"> "
                << std::endl;
        uint m = 0;
        char motorname[10];
        for (uint m = 0; m < idList.size(); m++) {
            sprintf(motorname, "motor%d", idList[m]);
            outfile << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
                    << controlmode[m] << "\" samplerate=\"" << samplingTime << "\">"
                    << std::endl;
            outfile << "<waypointlist>" << std::endl;
            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
                outfile << trajectories[idList[m]][i] << " ";
            outfile << "</waypointlist>" << std::endl;
            outfile << "</trajectory>" << std::endl;
        }
        outfile << "</roboybehavior>" << std::endl;
        outfile.close();
    }

    // set force to zero
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = 0.0f;
            else
                commandframe1[ganglion - 3].sp[motor] = 0.0f;
        }
    }
    updateCommandFrame();
    exchangeData();

    // restore controller types
    uint m = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            switch (motorControllerType_backup[m]) {
                case Position:
                    initPositionControl(ganglion, motor);
                    break;
                case Velocity:
                    initVelocityControl(ganglion, motor);
                    break;
                case Force:
                    initForceControl(ganglion, motor);
                    break;
            }
            m++;
        }
    }
    // restore setpoints
    i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = setPoints_backup[i];
            else
                commandframe1[ganglion - 3].sp[motor] = setPoints_backup[i];
            i++;
        }
    }
    updateCommandFrame();
    exchangeData();

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

float FlexRayHardwareInterface::recordTrajectories(
        float samplingTime, std::vector<std::vector<float>> &trajectories,
        std::vector<int> &idList, std::vector<int> &controlmode,
        int8_t *recording) {
    // this will be filled with the trajectories
    trajectories.resize(NUMBER_OF_GANGLIONS * NUMBER_OF_JOINTS_PER_GANGLION);

    // make a backup of control modes so after recording they can be restored
    std::vector<int8_t> motorControllerType_backup = motorControllerType;
    std::vector<float> setPoints_backup(NUMBER_OF_GANGLIONS *
                                        NUMBER_OF_JOINTS_PER_GANGLION);
    uint i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                setPoints_backup[i] = commandframe0[ganglion].sp[motor];
            else
                setPoints_backup[i] = commandframe1[ganglion - 3].sp[motor];
            i++;
        }
    }

    // force control is probably the most convenient mode for recording
    initForceControl();
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = 20.0f;
            else
                commandframe1[ganglion - 3].sp[motor] = 20.0f;
        }
    }
    updateCommandFrame();
    exchangeData();

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    while (*recording != STOP_TRAJECTORY) {
        dt = elapsedTime;
        if (*recording == PLAY_TRAJECTORY) {
            exchangeData();
            for (uint m = 0; m < idList.size(); m++) {
                if (motorState[idList[m]] == 1) { // only record if motor is available
                    if (controlmode[m] == 1)
                        trajectories.at(idList[m]).push_back(
                                GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                        .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                        .actuatorPos *
                                controlparams.radPerEncoderCount);
                    else if (controlmode[m] == 2)
                        trajectories.at(idList[m]).push_back(
                                GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                        .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                        .actuatorVel *
                                controlparams.radPerEncoderCount);
                    else if (controlmode[m] == 3) {
                        float polyPar[4];
                        polyPar[0] = 0;
                        polyPar[1] = 0.237536;
                        polyPar[2] = -0.000032;
                        polyPar[3] = 0;
                        float tendonDisplacement =
                                GanglionData[idList[m] / NUMBER_OF_JOINTS_PER_GANGLION]
                                        .muscleState[idList[m] % NUMBER_OF_JOINTS_PER_GANGLION]
                                        .tendonDisplacement;
                        float force = polyPar[0] + polyPar[1] * tendonDisplacement +
                                      polyPar[2] * powf(tendonDisplacement, 2.0f) +
                                      polyPar[3] * powf(tendonDisplacement, 3.0f);
                        trajectories.at(idList[m]).push_back(force);
                    }
                }
            }
            sample++;
        }
        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime) {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    }

    // set force to zero
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = 0.0f;
            else
                commandframe1[ganglion - 3].sp[motor] = 0.0f;
        }
    }
    updateCommandFrame();
    exchangeData();

    // restore controller types
    uint m = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < NUMBER_OF_JOINTS_PER_GANGLION; motor++) {
            if (motorState[m] == 1) {
                switch (motorControllerType_backup[m]) {
                    case Position:
                        initPositionControl(ganglion, motor);
                        break;
                    case Velocity:
                        initVelocityControl(ganglion, motor);
                        break;
                    case Force:
                        initForceControl(ganglion, motor);
                        break;
                }
            }
            m++;
        }
    }
    // restore setpoints
    i = 0;
    for (uint ganglion = 0; ganglion < NUMBER_OF_GANGLIONS; ganglion++) {
        for (uint motor = 0; motor < 4; motor++) {
            if (ganglion < 3)
                commandframe0[ganglion].sp[motor] = setPoints_backup[i];
            else
                commandframe1[ganglion - 3].sp[motor] = setPoints_backup[i];
            i++;
        }
    }
    updateCommandFrame();
    exchangeData();

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

DWORD FlexRayHardwareInterface::SPI_CSEnable(BYTE *OutputBuffer,
                                             DWORD *NumBytesToSend) {
    DWORD dwNumBytesToSend;
    dwNumBytesToSend = *NumBytesToSend;

    for (int loop = 0; loop < 2; loop++) // one 0x80 command can keep 0.2us, do 5
        // times to stay in this situation for
        // 1us
    {
        OutputBuffer[dwNumBytesToSend++] = '\x80'; // GPIO command for ADBUS
        OutputBuffer[dwNumBytesToSend++] = '\x00'; // set CS, MOSI and SCL low
        OutputBuffer[dwNumBytesToSend++] =
                '\x0b'; // bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }
    return dwNumBytesToSend;
}

DWORD FlexRayHardwareInterface::SPI_CSDisable(BYTE *OutputBuffer,
                                              DWORD *NumBytesToSend, bool end) {
    DWORD dwNumBytesToSend;
    dwNumBytesToSend = *NumBytesToSend;

    // one 0x80 command can keep 0.2us, do 5 times to stay in this situation for
    // 1us (leave CS low)
    // for(int loop=0;loop<2;loop++)
    {
        OutputBuffer[dwNumBytesToSend++] = '\x80'; // GPIO command for ADBUS
        OutputBuffer[dwNumBytesToSend++] = '\x00'; // set CS, MOSI and SCL low
        OutputBuffer[dwNumBytesToSend++] =
                '\x0b'; // bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }

    if (end == true) {
        // finally pull CS high
        OutputBuffer[dwNumBytesToSend++] = '\x80'; // GPIO command for ADBUS
        OutputBuffer[dwNumBytesToSend++] = '\x08'; // set CS high, MOSI and SCL low
        OutputBuffer[dwNumBytesToSend++] =
                '\x0b'; // bit3:CS, bit2:MISO, bit1:MOSI, bit0:SCK
    }

    return dwNumBytesToSend;
}

bool FlexRayHardwareInterface::CheckDeviceConnected(DWORD *NumDevs) {
    // -----------------------------------------------------------
    // Does an FTDI device exist?
    // -----------------------------------------------------------
    ROS_DEBUG("Checking for FTDI devices...");

    ftStatus = FT_CreateDeviceInfoList(NumDevs); // Get the number of FTDI devices
    if (ftStatus != FT_OK)                       // Did the command execute OK?
    {
        getErrorMessage(ftStatus, errorMessage);
        ROS_ERROR_STREAM(
                "Error in getting the number of devices, Error code: " << errorMessage);
        return false; // Exit with error
    }
    if (*NumDevs < 1) // Exit if we don't see any
    {
        ROS_WARN("There are no FTDI devices installed");
        return false; // Exit with error
    }
    ROS_DEBUG_STREAM(*NumDevs << " FTDI devices found-the count includes "
            "individual ports on a single chip");
    return true;
}

bool FlexRayHardwareInterface::GetDeviceInfo(DWORD *NumDevs) {
    FT_DEVICE_LIST_INFO_NODE *devInfo;

    // ------------------------------------------------------------
    // If yes then print details of devices
    // ------------------------------------------------------------
    devInfo = (FT_DEVICE_LIST_INFO_NODE *) malloc(
            sizeof(FT_DEVICE_LIST_INFO_NODE) * (*NumDevs));
    ftStatus = FT_GetDeviceInfoList(devInfo, NumDevs);
    if (ftStatus == FT_OK) {
        for (unsigned int i = 0; i < *NumDevs; i++) {
            ROS_DEBUG_STREAM(" Dev: " << i);
            ROS_DEBUG_STREAM(" Flags=0x" << devInfo[i].Flags);
            ROS_DEBUG_STREAM(" Type=0x" << devInfo[i].Type);
            ROS_DEBUG_STREAM(" ID=0x" << devInfo[i].ID);
            ROS_DEBUG_STREAM(" LocId=0x" << devInfo[i].LocId);
            ROS_DEBUG_STREAM(" SerialNumber=" << devInfo[i].SerialNumber);
            ROS_DEBUG_STREAM(" Description=" << devInfo[i].Description);
            ROS_DEBUG_STREAM(" ftHandle=0x" << devInfo[i].ftHandle);
        }
        return true;
    } else {
        ROS_WARN("Could not find info for devices");
        return false; // return with error
    }
}

bool FlexRayHardwareInterface::OpenPortAndConfigureMPSSE(
        FT_HANDLE *ftHandle, DWORD InTransferSize, DWORD OutTransferSize) {
    DWORD dwNumBytesToRead, dwNumBytesRead;
    BYTE byInputBuffer[DATASETSIZE *
                       2]; // Local buffer to hold data read from the FT2232H

    // -----------------------------------------------------------
    // Open the port on first device located
    // -----------------------------------------------------------
    ftStatus = FT_Open(0, ftHandle);
    if (ftStatus != FT_OK) {
        getErrorMessage(ftStatus, errorMessage);
        ROS_ERROR_STREAM("Open Failed with error " << errorMessage);
        return false; // Exit with error
    } else
        ROS_DEBUG("Port opened");

    // ------------------------------------------------------------
    // Configure MPSSE and test for synchronisation
    // ------------------------------------------------------------

    // Configure port parameters
    ROS_DEBUG("Configuring port for MPSSE use");
    ftStatus |= FT_ResetDevice(*ftHandle); // Reset USB device
    ftStatus |= FT_GetQueueStatus(
            *ftHandle, &dwNumBytesToRead); // Purge USB receive buffer first by
    // reading out all old data from FT2232H
    // receive buffer
    // Get the number of bytes in the FT2232H receive buffer
    if ((ftStatus == FT_OK) &&
        (dwNumBytesToRead >
         0)) // Read out the data from FT2232H receive buffer if not empty
        FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead, &dwNumBytesRead);
    else
        ROS_WARN("Buffer empty");

    // ftStatus |= FT_SetUSBParameters(ftHandle, 65536, 65535);  // Set USB
    // request transfer sizes to 64K
    ftStatus |= FT_SetUSBParameters(*ftHandle, InTransferSize,
                                    OutTransferSize); // Set USB request transfer
    // sizes...page 73 d2XX
    // programmer guide (only
    // dwInTransferSize has been
    // implemented)
    /*  FT_STATUS FT_SetUSBParameters(FT_HANDLE ftHandle, DWORD dwInTransferSize,
     DWORD dwOutTransferSize)
     (only dwInTransferSize currently supported..test this...see d2XX programmers
     guide p 73)
     */
    ftStatus |= FT_SetChars(*ftHandle, false, 0, false,
                            0); // Disable event and error characters
    /*  FT_STATUS FT_SetChars (FT_HANDLE ftHandle, UCHAR uEventCh, UCHAR
     uEventChEn,UCHAR uErrorCh, UCHAR uErrorChEn)
     (uEventCh = event character, uEventChEn = enable event character insertion,
     uErrorCh = error character, ...)
     */
    // ftStatus |= FT_SetTimeouts(ftHandle, 3000, 3000);         // Sets the read
    // and write timeouts in milliseconds
    ftStatus |= FT_SetTimeouts(
            *ftHandle, 300, 300); // Sets the read and write timeouts in milliseconds
    /*  FT_STATUS FT_SetTimeouts (FT_HANDLE ftHandle, DWORD dwReadTimeout, DWORD
     dwWriteTimeout)
     (dwReadTimeout, dwWriteTimeout in ms)
     */
    ftStatus |= FT_SetLatencyTimer(
            *ftHandle, 1); // Set the latency timer to 1mS (default is 16mS)
    /*  FT_STATUS FT_SetLatencyTimer (FT_HANDLE ftHandle, UCHAR ucTimer)
     (ucTimer = require latency value in milliseconds [range of 2-255 page 68 of
     d2XX programmers guide?])
     */
    // ftStatus |= FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x00, 0x00);
    // Turn on flow control to synchronize IN requests
    /*  FT_STATUS FT_SetFlowControl (FT_HANDLE ftHandle, USHORT usFlowControl,
     UCHAR uXon, UCHAR uXoff)
     (usFlowControl must be either FT_FLOW_NONE, FT_FLOW_RTS_CTS, FT_FLOW_DTR_DSR
     or FT_FLOW_XON_XOFF)
     (uXon = character to signal Xon [Only used is set to FT_FLOW_XON_XOFF])
     (uXoff = character to signal Xoff [Only used is set to FT_FLOW_XON_XOFF])
     */
    ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x00); // Reset controller
    /*  FT_STATUS FT_SetBitmode (FT_HANDLE ftHandle, UCHAR ucMask, UCHAR ucMode)
     (ucMask sets which bits are inputs [=0] or outputs [=1])
     (ucMode can one of the following:
     0x0 = Reset
     0x1 = Asynchronous Bit Bang
     0x2 = MPSSE (FT2232, FT2232H, FT4232H and FT232H devices only)
     0x4 = Synchronous Bit Bang (FT232R, FT245R, FT2232, FT2232H, FT4232H and
     FT232H devices only)
     0x8 = MCU Host Bus Emulation Mode (FT2232, FT2232H, FT4232H and FT232H
     devices only)
     0x10 = Fast Opto-Isolated Serial Mode (FT2232, FT2232H, FT4232H and FT232H
     devices only)
     0x20 = CBUS Bit Bang Mode (FT232R and FT232H devices only)
     0x40 = Single Channel Synchronous 245 FIFO Mode (FT2232H and FT232H devices
     only)
     )

     */
    ftStatus |= FT_SetBitMode(*ftHandle, 0x0, 0x02); // Enable MPSSE mode
    if (ftStatus != FT_OK) {
        getErrorMessage(ftStatus, errorMessage);
        ROS_ERROR_STREAM(
                "Error in initializing the MPSSE on device. Error: " << errorMessage);
        FT_Close(ftHandle);
        return false; // Exit with error
    }
    usleep(1); // Wait for all the USB stuff to complete and work

    ROS_DEBUG("MPSSE ready for commands");
    return true;
}

bool FlexRayHardwareInterface::TestMPSSE(FT_HANDLE *ftHandle) {
    BYTE byOutputBuffer[DATASETSIZE * 2];
    BYTE byInputBuffer[DATASETSIZE * 2];
    DWORD dwNumBytesToSend = 0;
    DWORD dwNumBytesSent, dwNumBytesRead, dwNumBytesToRead;

    /*
     *  Now the MPSSE is ready for commands, each command consists of an op-code
     * followed by any necessary parameters or
     *  data. Each op-code is sent using FT_Write call
     */

    /* Synchronisation and Bad communication detection */
    // Enable internal loop-back
    byOutputBuffer[dwNumBytesToSend++] = 0x84; // Enable loopback
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend,
                        &dwNumBytesSent); // Send off the loopback command
    dwNumBytesToSend = 0;                 // Reset output buffer pointer

    // Check the receive buffer - it should be empty
    ftStatus = FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead); // Get the number
    // of bytes in the
    // FT2232H receive
    // buffer
    if (dwNumBytesToRead != 0) {
        getErrorMessage(ftStatus, errorMessage);
        ROS_FATAL_STREAM(
                "Error - MPSSE receive buffer should be empty, Error Code: "
                        << errorMessage << ", try to run again!");
        FT_SetBitMode(*ftHandle, 0x0, 0x00); // Reset the port to disable MPSSE
        FT_Close(*ftHandle);                 // Close the USB port
        return false;                        // Exit with error
    } else
        ROS_DEBUG("Internal loop-back configured and receive buffer is empty");

    // send bad op-code to check every thing is working correctly
    byOutputBuffer[dwNumBytesToSend++] =
            0xAB; // Add bogus command ‘0xAB’ to the queue
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend,
                        &dwNumBytesSent); // Send off the BAD command
    dwNumBytesToSend = 0;                 // Reset output buffer pointer

    uint counter = 0;
    do {
        ftStatus =
                FT_GetQueueStatus(*ftHandle, &dwNumBytesToRead); // Get the number of
        // bytes in the device
        // input buffer
        counter++;
    } while ((dwNumBytesToRead == 0) && (ftStatus == FT_OK) &&
             (counter < 100)); // wait for bytes to return, an error or Timeout

    bool bCommandEchod = false;
    if (dwNumBytesToRead) {
        ftStatus = FT_Read(*ftHandle, &byInputBuffer, dwNumBytesToRead,
                           &dwNumBytesRead); // Read the data from input buffer
        for (DWORD dwCount = 0; dwCount < dwNumBytesRead - 1;
             dwCount++) // Check if Bad command and echo command are received
        {
            if ((byInputBuffer[dwCount] == 0xFA) &&
                (byInputBuffer[dwCount + 1] == 0xAB)) {
                bCommandEchod = true;
                break;
            }
        }
    }

    if (bCommandEchod == false) {
        ROS_ERROR("Error in synchronizing the MPSSE");
        FT_SetBitMode(*ftHandle, 0x0, 0x00); // Reset the port to disable MPSSE
        FT_Close(*ftHandle);                 // Close the USB port
        return false;                        // Exit with error
    } else {
        ROS_DEBUG("MPSSE synchronised.");
        byOutputBuffer[dwNumBytesToSend++] =
                '\x85'; // Command to turn off loop back of TDI/TDO connection
        ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend,
                            &dwNumBytesSent); // Send off the loopback command
        dwNumBytesToSend = 0;                 // Reset output buffer pointer
        return true;
    }
}

bool FlexRayHardwareInterface::ConfigureSPI(FT_HANDLE *ftHandle,
                                            DWORD dwClockDivisor) {
    BYTE byOutputBuffer[DATASETSIZE * 2];
    DWORD dwNumBytesToSend = 0;
    DWORD dwNumBytesSent;

    // ------------------------------------------------------------
    // Configure MPSSE for SPI communications
    // ------------------------------------------------------------
    // The SK clock frequency can be worked out as follows with divide by 5 set as
    // off:: SK frequency = 60MHz /((1 + [(1 + (0xValueH*256) OR 0xValueL])*2)

    byOutputBuffer[dwNumBytesToSend++] =
            '\x8A'; // Ensure disable clock divide by 5 for 60Mhz master clock
    byOutputBuffer[dwNumBytesToSend++] =
            '\x97'; // Ensure turn off adaptive clocking
    byOutputBuffer[dwNumBytesToSend++] = '\x8D'; // disable 3 phase data clock
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend,
                        &dwNumBytesSent); // Send out the commands
    if (ftStatus != FT_OK) {
        getErrorMessage(ftStatus, errorMessage);
        ROS_ERROR_STREAM("Error configuring SPI, Error code " << errorMessage);
        FT_SetBitMode(*ftHandle, 0x0, 0x00); // Reset the port to disable MPSSE
        FT_Close(*ftHandle);                 // Close the USB port
        return false;
    }

    dwNumBytesToSend = 0; // Clear output buffer
    byOutputBuffer[dwNumBytesToSend++] =
            '\x80'; // Set directions of lower 8 pins, set value on bits set as output
    byOutputBuffer[dwNumBytesToSend++] =
            '\x08'; // Set SCK, DO, DI low and CS high
    byOutputBuffer[dwNumBytesToSend++] =
            '\x0b'; // Set SK,DO,GPIOL0 pins as output =1, other pins as input=0
    byOutputBuffer[dwNumBytesToSend++] = '\x86'; // Command to set clock divisor
    byOutputBuffer[dwNumBytesToSend++] =
            (BYTE) (dwClockDivisor & '\xFF'); // Set 0xValueL of clock divisor
    byOutputBuffer[dwNumBytesToSend++] =
            (BYTE) (dwClockDivisor >> 8); // Set 0xValueH of clock divisor
    byOutputBuffer[dwNumBytesToSend++] =
            '\x85'; // Command to turn off loop back of TDI/TDO connection
    ftStatus = FT_Write(*ftHandle, byOutputBuffer, dwNumBytesToSend,
                        &dwNumBytesSent); // Send out the commands
    if (ftStatus != FT_OK) {
        getErrorMessage(ftStatus, errorMessage);
        ROS_ERROR_STREAM("Error configuring SPI " << errorMessage);
        FT_SetBitMode(*ftHandle, 0x0, 0x00); // Reset the port to disable MPSSE
        FT_Close(*ftHandle);                 // Close the USB port
        return false;
    }
    dwNumBytesToSend = 0; // Clear output buffer
    usleep(100);          // Delay for 100us
    ROS_DEBUG("SPI initialisation successful");
    return true;
}

BOOL FlexRayHardwareInterface::SPI_WriteByte(FT_HANDLE ftHandle, WORD bdata) {
    DWORD dwNumBytesSent;
    DWORD dwNumBytesToSend = 0;
    BYTE OutputBuffer[512];

    dwNumBytesToSend = SPI_CSEnable(&OutputBuffer[0], &dwNumBytesToSend);
    OutputBuffer[dwNumBytesToSend++] =
            MSB_FALLING_EDGE_CLOCK_BYTE_OUT; // Byte out falling edge msb first
    OutputBuffer[dwNumBytesToSend++] =
            1; // data length in bytes (low byte)  } +1!!
    OutputBuffer[dwNumBytesToSend++] = 0; // data length in bytes (high byte) }
    OutputBuffer[dwNumBytesToSend++] = bdata >> 8;   // output high byte
    OutputBuffer[dwNumBytesToSend++] = bdata & 0xff; // output low byte
    dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
    ftStatus =
            FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend, &dwNumBytesSent);
    // send out MPSSE command to MPSSE engine
    ROS_DEBUG_STREAM(dwNumBytesSent << " bytes sent through SPI");
    return ftStatus;
}

FT_STATUS FlexRayHardwareInterface::SPI_WriteBuffer(FT_HANDLE ftHandle,
                                                    WORD *buffer,
                                                    DWORD numwords) {
    DWORD dwNumBytesSent = 0;
    DWORD dwNumBytesToSend = 0;
    BYTE OutputBuffer[USBOUTSIZE];
    WORD bdata;

    // Bytes actually sent = data words*2 + 3 Commands*data words + 3
    // CSenable*data words + 6 CSdisable*data words => 14* Data words
    for (unsigned int i = 0; i < numwords; i++) {
        bdata = buffer[i];
        dwNumBytesToSend = SPI_CSEnable(&OutputBuffer[0], &dwNumBytesToSend);
        OutputBuffer[dwNumBytesToSend++] =
                MSB_FALLING_EDGE_OUT_RISING_EDGE_IN_BYTE; // MSB_RISING_EDGE_OUT_FALLING_EDGE_IN_BYTE;//
        OutputBuffer[dwNumBytesToSend++] =
                1; // data length in bytes (low byte)  } +1!!
        OutputBuffer[dwNumBytesToSend++] = 0; // data length in bytes (high byte) }
        OutputBuffer[dwNumBytesToSend++] = (BYTE) (bdata >> 8);   // output high byte
        OutputBuffer[dwNumBytesToSend++] = (BYTE) (bdata & 0xff); // output low byte
        dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
    }
// dwNumBytesToSend = SPI_CSDisable(&OutputBuffer[0], &dwNumBytesToSend, true);
// printf("%i NumBytesToSend", sizeof(OutputBuffer));
#ifdef HARDWARE
    do {
        ftStatus =
                FT_Write(ftHandle, OutputBuffer, dwNumBytesToSend,
                         &dwNumBytesSent); // send out MPSSE command to MPSSE engine
        if (ftStatus != FT_OK) {
            getErrorMessage(ftStatus, errorMessage);
            ROS_ERROR_STREAM(" something wrong with FT_Write call, Error code "
                                     << errorMessage);
        }
        // else
        //  std::cout << dwNumBytesSent <<" bytes sent through SPI" << std::endl;
    } while (ftStatus != FT_OK);
#else
    ROS_DEBUG("No Hardware mode enabled, not trying to send anything");
#endif
    dwNumBytesToSend = 0; // Clear output buffer
    return ftStatus;
}
