#include "FlexRayUSBInterface.h"
#include <QApplication>

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    FlexRayUSBInterface interface(100);
//    sint8 controlMode[] = {0,0,0,0};
//    sint8 operationMode[] = {0,0,0,0};
//    float32 sp[] = {0.5,0.5,0.5,0.5};
    
    app.exec();
    return 0;
}
