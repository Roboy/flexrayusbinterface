#include "FlexRayUSBInterface.h"
#include <QApplication>

using namespace eu::myode::myorobot;

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    FlexRayUSBInterface interface(100);
    return 0;
}
