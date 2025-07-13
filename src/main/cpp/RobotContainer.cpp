// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "Constants.h"
#include "subsystems/SubDrivebase.h"
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer(ctre::phoenix::motorcontrol::can::WPI_VictorSPX& coral_motor)
    : m_coralMotor(coral_motor) {

  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  ConfigureBindings();
}


void RobotContainer::ConfigureBindings() {
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // drive forward 3sec
  auto driveCommand = SubDrivebase::GetInstance().Drive(
    [] { return frc::ChassisSpeeds{0.65_mps, 0_mps, 0_tps}; }, false
  ).WithTimeout(4_s);

  // coral motor brrr
  auto motorCommand = frc2::StartEndCommand(
    [this] { m_coralMotor.Set(-0.5); },
    [this] { m_coralMotor.Set(0.0); },
    {}
  ).WithTimeout(2_s);

  // do stuff i tell u to/
  return frc2::cmd::Sequence(
    std::move(driveCommand),
    std::move(motorCommand)
  );
}
