#pragma once
#include "rev/CANSparkMax.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "Falcon500.h"
#include "angleOptimization.h"
#include "Parameters.h"
#include "complex"

using namespace std;

// controls the motion of each swerve module
class SwerveModule
{
private:
    complex<float> steeringVector; // module drive vector for steering the robot clockwise
    Falcon500 *driveMotor; // drive motor object
    rev::CANSparkMax *turningMotor; // wheel swiveling motor object
    hardware::CANcoder *wheelAngleEncoder; // measures the absolute angle of the wheel
    float lastPosition = 0; // last position of the drive motor
    complex<float> positionChangeVector; // vector defining module's position change since last Set() call

public:
    SwerveModule(int driveMotorCANID, int turningMotorCANID, int canCoderID, complex<float> position)
    {
        driveMotor = new Falcon500{driveMotorCANID};
        turningMotor = new rev::CANSparkMax{turningMotorCANID, rev::CANSparkMax::MotorType::kBrushless};
        wheelAngleEncoder = new hardware::CANcoder{canCoderID};
        // calculate the steering vector
        steeringVector = position;
         // rotate by -pi/2 radians
        steeringVector *= complex<float>(0, -1);
        steeringVector /= abs(steeringVector);
    }

    // initialize the drive motor and invert the turning motor
    void initialize()
    {
        driveMotor->initialize();
        turningMotor->SetInverted(true);
        turningMotor->BurnFlash();
    }

    // calculate the swerve module vector
    complex<float> getModuleVector(complex<float> driveRate, float turnRate)
    {
        return driveRate + steeringVector * turnRate;
    }

    /**
     * set this swerve module to its corresponding drive rate
     * 
     * arguments:
     * driveRate - robot-centric drive rate
     * angularRate - rate to spin the robot
    */ 
    void Set(complex<float> driveRate, float angularRate)
    {
        // find the current wheel angle
        float currentWheelAngle = M_PI/180 * wheelAngleEncoder->GetAbsolutePosition().GetValue().value();
        // find the module target velocity
        complex<float> moduleTargetVelocity = getModuleVector(driveRate, angularRate);
        // find the wheel's error from it's target angle
        float error = angleDifference(angleDifference(arg(moduleTargetVelocity), M_PI/2), currentWheelAngle);
        // find the drive motor velocity
        float driveMotorVelocity = abs(moduleTargetVelocity);
        // reverse the wheel direction if it is more efficient
        if (abs(error) > 90)
        {
            driveMotorVelocity = -driveMotorVelocity;
            error = angleSum(error, M_PI);
        }
        driveMotor->SetVelocity(driveMotorVelocity);
        // set the turning motor to a speed proportional to its error
        turningMotor->Set(error / M_PI);
        // find the delta position change since last Set() call
        float currentPosition = driveMotor->getPosition();
        positionChangeVector = polar<float>((currentPosition - lastPosition) * parameters.driveMotorInPerRot, M_PI/2-currentWheelAngle);
        lastPosition = currentPosition;
    }

    // gets this module's position change, useful for calculating robot position
    complex<float> getPositionChangeVector()
    {
        return positionChangeVector;
    }
};