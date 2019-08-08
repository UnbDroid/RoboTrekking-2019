#pragma once

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string>

#include "Pos.h"

class Robot
{
private:
    Pos position;
    float angle;

public:
    Robot(Pos position);
    ~Robot();
};