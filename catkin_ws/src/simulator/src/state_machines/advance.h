
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

void advance(movement *movements, float max_advance)
{

    movements->advance = max_advance;
    movements->twist = 0;

    fputs("Dentro de advance", stderr);
}
