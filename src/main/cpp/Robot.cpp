// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/SubDrivebase.h"

Robot::Robot()
  : coral_motor{canid::CORAL_OUTPUT_MOTOR},
  m_container(coral_motor) {

  }

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  SubDrivebase::GetInstance().SyncSensors();

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  if (m_container._driverController.GetRightTriggerAxis() > 0.9) {
    coral_motor.Set(-0.6);
  } else if (m_container._driverController.GetLeftTriggerAxis() > 0.9) {
    coral_motor.Set(0.3);
  } else {
    coral_motor.Set(0.0);
  }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
