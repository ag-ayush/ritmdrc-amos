/*
 * File:    main.cpp
 * Author:  James Letendre
 *
 * Main code loop for base package
 */
#include "base.h"
#include <ros/ros.h>

int main( int argc, char **argv )
{
    ros::init( argc, argv, "base" );

    Base base;

    base.loop();

    return 0;
}
