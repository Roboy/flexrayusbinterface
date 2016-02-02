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
        }
		ncurse.querySensoryData();
    }while( cmd != '9');
    return 0;
}
