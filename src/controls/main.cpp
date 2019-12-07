#include "robotcontrols.h"
#include <signal.h> 

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    signal(SIGINT,RobotControls::signalHandler);
    RobotControls robotControls;
    robotControls.go();
}

