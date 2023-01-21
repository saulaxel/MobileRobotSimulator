
/************************************************
 *                                              *
 *      advance.h                               *
 *                                              *
 *      Diego Cordero                           *
 *      Jesus Savage                            *
 *                                              *
 *                                              *
 *              Bio-Robotics Laboratory         *
 *              UNAM, 2019                      *
 *                                              *
 *                                              *
 ************************************************/


#include <stdio.h>


#include "../utilities/simulator_structures.h"

void turn(movement *movements, float max_turn_angle)
{
    movements->advance = 0;
    movements->twist = max_turn_angle;
    fputs("Dentro de turn()", stderr);
}
