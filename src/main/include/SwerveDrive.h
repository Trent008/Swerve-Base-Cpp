#pragma once
#include "AHRS.h"
#include "SwerveModule.h"

using namespace std;

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

    // NavX V2 gyro object
    AHRS navx{frc::SPI::Port::kMXP};
    
    // drive and turn rates with drive rates stored as complex numbers
    complex<float> currentDriveRate;
    complex<float> targetDriveRate;
    float currentTurnRate = 0;
    float targetTurnRate;

    // current position on the field relative to the starting position
    complex<float> currentPosition = parameters.startingPosition;
    // current angle
    float currentAngle;

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

    // set field centric drive rate
    void Set(complex<float> driveRate, float turnRate, bool useAcceleration = true)
    {
        // set the current angle to the gyro angle + the starting angle
        currentAngle = angleSum(-navx.GetYaw()*M_PI/180, parameters.startingAngle);

        // set the target rates to the input
        targetDriveRate = driveRate;
        targetTurnRate = turnRate;

        // modify the target rates to make them physically achievable
        normalizeSwerveRate(driveRate, turnRate);

        if (useAcceleration)
        {
            // find rate errors
            complex<float> driveRateError = targetDriveRate - currentDriveRate;
            float turnRateError = targetTurnRate - currentTurnRate;
            // limit increment to slew rate
            if (abs(driveRateError) > parameters.slewRate)
            {
                driveRateError *= parameters.slewRate / abs(driveRateError);
            }
            if (abs(turnRateError) > parameters.slewRate)
            {
                turnRateError *= parameters.slewRate / abs(turnRateError);
            }

            // increment current rates toward target rates
            currentDriveRate += driveRateError;
            currentTurnRate += turnRateError;
        }
        else
        {
            // set the drive rates directly from the input
            currentDriveRate = targetDriveRate;
            currentTurnRate = targetTurnRate;
        }

        // robot-orient the drive rate
        currentDriveRate *= polar<float>(1, -currentAngle);
        // stores the robot's change in position since last Set()
        complex<float> positionChange;
        // drive the modules and average the module position changes
        for (int i = 0; i < 4; i++)
        {
            modules[i].Set(currentDriveRate, currentTurnRate);
            positionChange += modules[i].getPositionChangeVector();
        }
        // field-orient the position change vector
        positionChange *= polar<float>(1, currentAngle);
        // average the position change of all four swerve modules
        positionChange /= 4;
        // add the change in position over this cycle to the running total
        currentPosition += positionChange;
    }

    // drive toward a point and angle (radians) and return true when the point is reached
    bool driveToward(complex<float> targetPostition, float targetAngle, float positionTolerance = 2, float angleTolerance = 5)
    {
        complex<float> positionError = targetPostition - currentPosition;
        float angleError = angleDifference(currentAngle, targetAngle);
        complex<float> positionPIDOutput = positionError * parameters.autoPositionP;
        float anglePIDOutput = angleError * parameters.autoAngleP;
        if (abs(positionPIDOutput) > parameters.autoMaxDriveRate)
        {
            positionPIDOutput *= parameters.autoMaxDriveRate / abs(positionPIDOutput);
        }
        if (abs(anglePIDOutput) > parameters.autoMaxTurnRate)
        {
            anglePIDOutput *= parameters.autoMaxTurnRate / abs(anglePIDOutput);
        }
        Set(positionPIDOutput, anglePIDOutput, false);
        return (abs(positionError) < positionTolerance) && (abs(angleError) < angleTolerance);
    }

    // limit the driving inputs to physically achievable values
    void normalizeSwerveRate(complex<float> driveRate, float turnRate)
    {
        // robot-orient the drive rate
        driveRate *= polar<float>(1, -currentAngle);

        // compare all of the module velocities to find the largest
        float fastestModule = 1;
        for (int i = 0; i < 4; i++) 
        {
            float moduleSpeed = abs(modules[i].getModuleVector(driveRate, currentTurnRate));
            if (moduleSpeed > fastestModule)
            {
                fastestModule = moduleSpeed;
            }
        }
        // scale down the targets so that the fastest module speed is 1
        targetDriveRate /= fastestModule;
        targetTurnRate /= fastestModule;
    }
} swerve;