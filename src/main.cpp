#include "FlexRayUSBInterface.h"
#include <QApplication>

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    FlexRayUSBInterface interface(100);
    return 0;
}
