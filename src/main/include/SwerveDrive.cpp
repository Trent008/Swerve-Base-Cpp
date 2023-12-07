#pragma once
#include "AHRS.h"
#include "SwerveModule.cpp"

// runs a swerve chassis
class SwerveDrive
{

private:
    // array of four swerve module objects with their positions relative to the center specified
    SwerveModule modules[4] =
        {
            SwerveModule{11, 31, 21, {-17.75, 25}},
            SwerveModule{12, 32, 22, {-17.75, -25}},
            SwerveModule{13, 33, 23, {17.75, 25}},
            SwerveModule{14, 34, 24, {17.75, -25}}};

    // NavX V2 object
    AHRS navx{frc::SPI::Port::kMXP};
    
    // stores current field rate setpoint
    Vector currentFieldRate;
    // stores current angular rate setpoint
    float currentAngularRate = 0;
    // current position on the field relative to the starting position
    Vector currentFieldPosition = parameters.startingPosition;
    // current angle on the field
    float currentFieldAngle;

public:
    // initialize the swerve modules and zero the NavX yaw
    void initialize()
    {
        for (int i = 0; i < 4; i++)
        {
            modules[i].initialize();
        }
        // navx.ZeroYaw();
    }

    // set the Swerve chassis field-centric drive rate
    void Set(Vector targetFieldRate, float targetAngularRate, bool useAcceleration = true)
    {
        // set the current field angle to the gyro angle + the starting angle
        currentFieldAngle = angleSum(navx.GetYaw(), parameters.startingAngle);
        // robot-orient the drive command
        targetFieldRate.rotateCW(-currentFieldAngle);
        // keep the module speeds <= 1
        normalizeSwerveRate(targetFieldRate, targetAngularRate);
        // field-orient the drive rate command again
        targetFieldRate.rotateCW(currentFieldAngle);

        if (useAcceleration)
        {
            // find the robot field drive rate error
            Vector positionalAccelIncrement = targetFieldRate.getSubtracted(currentFieldRate).getScaled(0.5);
            float angularAccelIncrement = (targetAngularRate - currentAngularRate) * 0.5;
            // limit increments to max acceleration rate
            if (t2D::abs(positionalAccelIncrement) > parameters.maxPercentChangePerCycle)
            {
                positionalAccelIncrement.scale(parameters.maxPercentChangePerCycle / t2D::abs(positionalAccelIncrement));
            }
            if (std::abs(angularAccelIncrement) > parameters.maxPercentChangePerCycle)
            {
                angularAccelIncrement *= parameters.maxPercentChangePerCycle / std::abs(angularAccelIncrement);
            }

            // increment from the current field rate
            currentFieldRate.add(positionalAccelIncrement);
            currentAngularRate += angularAccelIncrement;
        }
        else
        {
            // set the drive rates directly from the input
            currentFieldRate = targetFieldRate;
            currentAngularRate = targetAngularRate;
        }
        // stores the robot's change in position since last Set()
        Vector fieldPositionChange;
        // drive the modules and average the module position changes
        for (int i = 0; i < 4; i++)
        {
            modules[i].Set(currentFieldRate.getRotatedCW(-currentFieldAngle), currentAngularRate);
            fieldPositionChange.add(modules[i].getPositionChangeVector());
        }
        // orient the position change vector in the direction of robot motion
        fieldPositionChange.rotateCW(currentFieldAngle);
        // average the position change of all four swerve modules
        fieldPositionChange.divide(4);
        // add the change in position over this cycle to the running total
        currentFieldPosition.add(fieldPositionChange);
    }

    // drives the swerve drive toward a point and returns true when the point is reached
    bool driveToward(Vector targetPostition, float targetAngle, float positionTolerance = 2, float angleTolerance = 5)
    {
        Vector positionError = targetPostition.getSubtracted(currentFieldPosition);
        float angleError = angleDifference(currentFieldAngle, targetAngle);
        Vector positionPIDOutput = positionError.getScaled(parameters.autoPositionP);
        float anglePIDOutput = angleError * parameters.autoAngleP;
        if (t2D::abs(positionPIDOutput) > parameters.autoMaxDriveRate)
        {
            positionPIDOutput.scale(parameters.autoMaxDriveRate/t2D::abs(positionPIDOutput));
        }
        if (std::abs(anglePIDOutput) > parameters.autoMaxRotationRate)
        {
            anglePIDOutput *= parameters.autoMaxRotationRate/std::abs(anglePIDOutput);
        }
        Set(positionPIDOutput, anglePIDOutput, false);
        return (t2D::abs(positionError) < positionTolerance) && (std::abs(angleError) < angleTolerance);
    }

    // limit the driving inputs to physically achievable values
    void normalizeSwerveRate(Vector &driveRate, float &currentAngularRate)
    {
        float fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            float moduleSpeed = t2D::abs(modules[i].getModuleVector(driveRate, currentAngularRate));
            if (moduleSpeed > fastestModule)
            {
                fastestModule = moduleSpeed;
            }
        }
        driveRate.divide(fastestModule);
        currentAngularRate /= fastestModule;
    }
} swerve;