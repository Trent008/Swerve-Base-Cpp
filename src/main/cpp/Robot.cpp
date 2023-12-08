// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
  swerve.initialize();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic()
{
  // drive the robot in a 24 inch radius circle
  if (swerve.driveToward(polar<float>(i*M_PI/180, 24), 0))
  {
    i++;
  }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  swerve.Set(complex<float>(float(xBoxC.GetRawAxis(0)), -float(xBoxC.GetRawAxis(1))), -float(xBoxC.GetRawAxis(4)));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
