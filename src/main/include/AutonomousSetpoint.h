#pragma once
#include "complex"

using namespace std;

struct AutonomousSetpoint
{
    complex<float> position;
    float angle = 0;
};
