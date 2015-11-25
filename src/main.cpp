//#include "FlexRayUSBInterface.h"
#include "FlexRayHardwareInterface.hpp"
INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[]){
    START_EASYLOGGINGPP(argc, argv);
    // Load configuration from file
    el::Configurations conf("logging.conf");
    // Actually reconfigure all loggers instead
    el::Loggers::reconfigureAllLoggers(conf);
    
    FlexRayHardwareInterface interface;
    while(1){
        double pos;
        LOG(INFO) << "Set position (rad):";
        std::cin >> pos;
        LOG(INFO) << pos;
        interface.commandframe[0].sp[0] = pos;
        interface.sendCommandFrame(interface.commandframe,interface.commandframe,&interface.controlparams);
        LOG(INFO)   << "Motor Info: "  << std::endl
                    << "jointPos: " << interface.GanglionData[0].muscleState[0].jointPos << std::endl 
                    << "actuatorPos (encoder counts): " << interface.GanglionData[0].muscleState[0].actuatorPos << std::endl
                    << "actuatorVel (counts/s): " << interface.GanglionData[0].muscleState[0].actuatorVel << std::endl
                    << "actuatorCurrent: " << interface.GanglionData[0].muscleState[0].actuatorCurrent << std::endl
                    << "tendonDisplacement: " << interface.GanglionData[0].muscleState[0].tendonDisplacement;
    }

    return 0;
}
