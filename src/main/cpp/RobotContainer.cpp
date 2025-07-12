// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "Constants.h"
#include "subsystems/SubDrivebase.h"

RobotContainer::RobotContainer() {
  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));

  // Trigger Bindings
  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  
  return frc2::cmd::None();
}

