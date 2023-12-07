#pragma once
#include "rev/CANSparkMax.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "Falcon500.cpp"
#include "angleOptimization.cpp"
#include "Parameters.cpp"

// controls the motion of each swerve module
class SwerveModule
{
private:
    Vector steeringVector; // module drive vector for steering the robot clockwise
    Falcon500 *driveMotor; // drive motor object
    rev::CANSparkMax *turningMotor; // wheel swiveling motor object
    hardware::CANcoder *wheelAngleEncoder; // measures the absolute angle of the wheel
    float lastPosition = 0; // last position of the drive motor
    Vector positionChangeVector; // vector defining module's position change since last Set() call

public:
    SwerveModule(int driveMotorCANID, int turningMotorCANID, int canCoderID, Vector position)
    {
        driveMotor = new Falcon500{driveMotorCANID};
        turningMotor = new rev::CANSparkMax{turningMotorCANID, rev::CANSparkMax::MotorType::kBrushless};
        wheelAngleEncoder = new hardware::CANcoder{canCoderID};
        // calculate the steering vector
        steeringVector = position;
        steeringVector.rotateCW(90);
        steeringVector.divide(t2D::abs(steeringVector));
    }

    // initialize the drive motor and invert the turning motor
    void initialize()
    {
        driveMotor->initialize();
        turningMotor->SetInverted(true);
        turningMotor->BurnFlash();
    }

    // calculate the swerve module vector
    Vector getModuleVector(Vector driveRate, float angularRate)
    {
        return driveRate.getAdded(steeringVector.getScaled(angularRate));
    }

    /**
     * set this swerve module to its corresponding drive rate
     * 
     * arguments:
     * driveRate - robot-centric drive rate
     * angularRate - rate to spin the robot
    */ 
    void Set(Vector driveRate, float angularRate)
    {
        // find the current wheel angle
        float currentWheelAngle = wheelAngleEncoder->GetAbsolutePosition().GetValue().value();
        // find the module target velocity
        Vector moduleTargetVelocity = getModuleVector(driveRate, angularRate);
        // find the wheel's error from it's target angle
        float error = angleDifference(moduleTargetVelocity.getAngle(), currentWheelAngle);
        // find the drive motor velocity
        float driveMotorVelocity = t2D::abs(moduleTargetVelocity);
        // reverse the wheel direction if it is more efficient
        if (std::abs(error) > 90)
        {
            driveMotorVelocity = -driveMotorVelocity;
            error = angleSum(error, 180);
        }
        driveMotor->SetVelocity(driveMotorVelocity);
        // set the turning motor to a speed proportional to its error
        turningMotor->Set(error / 180);
        // find the delta position change since last Set() call
        float currentPosition = driveMotor->getPosition();
        positionChangeVector = Vector{0, (currentPosition - lastPosition) * parameters.driveMotorInchesPerRotation}.getRotatedCW(currentWheelAngle);
        lastPosition = currentPosition;
    }

    // gets this module's position change, useful for calculating robot position
    Vector getPositionChangeVector()
    {
        return positionChangeVector;
    }
};