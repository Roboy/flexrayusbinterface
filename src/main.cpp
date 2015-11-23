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

    return 0;
}
