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
            SwerveModule{11, 31, 21, (-17.75, 25)},
            SwerveModule{12, 32, 22, (-17.75, -25)},
            SwerveModule{13, 33, 23, (17.75, 25)},
            SwerveModule{14, 34, 24, (17.75, -25)}};

    // NavX V2 object
    AHRS navx{frc::SPI::Port::kMXP};
    
    // stores current field rate setpoint
    complex<float> currentFieldRate;
    // stores current angular rate setpoint
    float currentAngularRate = 0;
    // current position on the field relative to the starting position
    complex<float> currentFieldPosition = parameters.startingPosition;
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
        navx.ZeroYaw();
    }

    // set the Swerve chassis field-centric drive rate
    void Set(complex<float> targetFieldRate, float targetAngularRate, bool useAcceleration = true)
    {
        // set the current field angle to the gyro angle + the starting angle
        currentFieldAngle = angleSum(navx.GetYaw()*M_PI/180, parameters.startingAngle);
        // robot-orient the drive command
        targetFieldRate *= polar<float>(1, currentFieldAngle);
        // keep the module speeds <= 1
        normalizeSwerveRate(targetFieldRate, targetAngularRate);
        // field-orient the drive rate command again
        targetFieldRate *= polar<float>(1, -currentFieldAngle);

        if (useAcceleration)
        {
            // find the robot field drive rate error
            complex<float> positionalAccelIncrement = (targetFieldRate - currentFieldRate) * float(0.5);
            float angularAccelIncrement = (targetAngularRate - currentAngularRate) * float(0.5);
            // limit increments to max acceleration rate
            if (abs(positionalAccelIncrement) > parameters.maxPercentChangePerCycle)
            {
                positionalAccelIncrement *= parameters.maxPercentChangePerCycle / abs(positionalAccelIncrement);
            }
            if (abs(angularAccelIncrement) > parameters.maxPercentChangePerCycle)
            {
                angularAccelIncrement *= parameters.maxPercentChangePerCycle / abs(angularAccelIncrement);
            }

            // increment from the current field rate
            currentFieldRate += positionalAccelIncrement;
            currentAngularRate += angularAccelIncrement;
        }
        else
        {
            // set the drive rates directly from the input
            currentFieldRate = targetFieldRate;
            currentAngularRate = targetAngularRate;
        }
        // stores the robot's change in position since last Set()
        complex<float> fieldPositionChange;
        // drive the modules and average the module position changes
        for (int i = 0; i < 4; i++)
        {
            modules[i].Set(currentFieldRate * polar<float>(1, currentFieldAngle), currentAngularRate);
            fieldPositionChange += modules[i].getPositionChangeVector();
        }
        // orient the position change vector in the direction of robot motion
        fieldPositionChange *= polar<float>(1, -currentFieldAngle);
        // average the position change of all four swerve modules
        fieldPositionChange /= 4;
        // add the change in position over this cycle to the running total
        currentFieldPosition += fieldPositionChange;
    }

    // drives the swerve drive toward a point and returns true when the point is reached
    bool driveToward(complex<float> targetPostition, float targetAngle, float positionTolerance = 2, float angleTolerance = 5)
    {
        complex<float> positionError = targetPostition - currentFieldPosition;
        float angleError = angleDifference(currentFieldAngle, targetAngle);
        complex<float> positionPIDOutput = positionError * parameters.autoPositionP;
        float anglePIDOutput = angleError * parameters.autoAngleP;
        if (abs(positionPIDOutput) > parameters.autoMaxDriveRate)
        {
            positionPIDOutput *= parameters.autoMaxDriveRate / abs(positionPIDOutput);
        }
        if (abs(anglePIDOutput) > parameters.autoMaxRotationRate)
        {
            anglePIDOutput *= parameters.autoMaxRotationRate / abs(anglePIDOutput);
        }
        Set(positionPIDOutput, anglePIDOutput, false);
        return (abs(positionError) < positionTolerance) && (abs(angleError) < angleTolerance);
    }

    // limit the driving inputs to physically achievable values
    void normalizeSwerveRate(complex<float> &driveRate, float &currentAngularRate)
    {
        float fastestModule = 1;
        for (int i = 0; i < 4; i++) // compare all of the module velocities to find the largest
        {
            float moduleSpeed = abs(modules[i].getModuleVector(driveRate, currentAngularRate));
            if (moduleSpeed > fastestModule)
            {
                fastestModule = moduleSpeed;
            }
        }
        driveRate /= fastestModule;
        currentAngularRate /= fastestModule;
    }
} swerve;