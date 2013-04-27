#include "comm.h"
#include "motor.h"

void setup()
{
    // set up motors
    Motor::setup();
    // set up ros messages
    comm_setup();
    // enable interrupt
    sei();
}

void loop()
{
    // run motor tasks
    Motor::run();
    // handle ros messages
    comm_run();
}
