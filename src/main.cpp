#include <cstdlib>
#include <ncurses_flexray.hpp>

int main(int argc, char* argv[]){

	NCurses_flexray ncurse;
	char cmd;
	noecho();
    do{
		timeout(10);
        cmd = mvgetch(4,0);
        switch (cmd){
            case '0':
				ncurse.positionControl();
                break;
            case '1':
				ncurse.velocityControl();
                break;
            case '2':
				ncurse.forceControl();
                break;
			case '3':
				ncurse.switchMotor();
				break;
			case '4':
				ncurse.measureConnection();
				break;
			case '5':
				ncurse.recordTrajectories();
				break;
			case '6':
				ncurse.setAllToForce();
				break;
			case '7':
				ncurse.resetControl();
				break;
			case '8':
                ncurse.resetAll();
                break;
            case 'r':
                ncurse.remoteAndroidControl();
                break;
            case 'p':
                ncurse.publishMotorInfo();
                break;
        }
		ncurse.querySensoryData();
    }while( cmd != '9');
    return 0;
}
