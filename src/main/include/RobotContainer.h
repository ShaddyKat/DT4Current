#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

class RobotContainer {
 public:
  // Constructor taking motor reference
  explicit RobotContainer(ctre::phoenix::motorcontrol::can::WPI_VictorSPX& coral_motor);

  frc2::CommandXboxController _driverController{0};

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  // Store motor reference here:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX& m_coralMotor;
};
