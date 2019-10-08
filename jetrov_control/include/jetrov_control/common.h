#ifndef JETROV_CONTROL_COMMON_H
#define JETROV_CONTROL_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "jetrov_control/JHPWMPCA9685.h"
#include "jetrov_control/const.h"

namespace jetrov_control{

int getkey()
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int map(double x, int in_min, int in_max, int out_min, int out_max)
{
    int toReturn = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;

    return toReturn ;
}

}  // end namespace jetrov_control

#endif // JETROV_CONTROL_COMMON_H
