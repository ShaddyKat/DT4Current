#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <units/time.h>
#include "RobotContainer.h"
#include <frc/geometry/Translation2d.h>

SubDrivebase::SubDrivebase() {
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Rotation Controller",
                               &_teleopRotationController);
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Translation Controller",
                               &_teleopTranslationController);
  _teleopRotationController.EnableContinuousInput(0_deg, 360_deg);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);

}

void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();

  // 0.04121451348939883
  // 4.465m

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  UpdateOdometry();
  frc::SmartDashboard::PutNumber("Drivebase/loop time (sec)", (frc::GetTime() - loopStart).value());
}

void SubDrivebase::SimulationPeriodic() {
  
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  std::string configPath = "Drivebase/Config/";
  auto deadband = frc::SmartDashboard::GetNumber(configPath + "Joystick Deadband", JOYSTICK_DEADBAND);
  auto maxVelocity = frc::SmartDashboard::GetNumber(configPath + "Max Velocity", MAX_VELOCITY.value());
  auto maxAngularVelocity = frc::SmartDashboard::GetNumber(configPath + "Max Angular Velocity", MAX_ANGULAR_VELOCITY.value());
  auto maxJoystickAccel = frc::SmartDashboard::GetNumber(configPath + "Max Joystick Accel", MAX_JOYSTICK_ACCEL);
  auto maxAngularJoystickAccel =
      frc::SmartDashboard::GetNumber(configPath + "Max Joystick Angular Accel", MAX_ANGULAR_JOYSTICK_ACCEL);
  auto translationScaling =
      frc::SmartDashboard::GetNumber(configPath + "Translation Scaling", TRANSLATION_SCALING);
  auto rotationScaling = frc::SmartDashboard::GetNumber(configPath + "Rotation Scaling", ROTATION_SCALING);

  // Recreate slew rate limiters if limits have changed
  if (maxJoystickAccel != _tunedMaxJoystickAccel) {
    _xStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _yStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _tunedMaxJoystickAccel = maxJoystickAccel;
  }
  if (maxAngularJoystickAccel != _tunedMaxAngularJoystickAccel) {
    _rotStickLimiter = frc::SlewRateLimiter<units::scalar>{maxAngularJoystickAccel / 1_s};
    _tunedMaxAngularJoystickAccel = maxAngularJoystickAccel;
  }

  // Apply deadbands
  double rawTranslationY = frc::ApplyDeadband(-controller.GetLeftY(), deadband);
  double rawTranslationX = frc::ApplyDeadband(-controller.GetLeftX(), deadband);
  double rawRotation = frc::ApplyDeadband(-controller.GetRightX(), deadband);

  // Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
  double rawTranslationR = std::min(1.0, sqrt(pow(rawTranslationX, 2) + pow(rawTranslationY, 2)));
  double translationTheta = atan2(rawTranslationY, rawTranslationX);
  double scaledTranslationR = pow(rawTranslationR, translationScaling);

  // Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
  double scaledTranslationY = scaledTranslationR * sin(translationTheta);
  double scaledTranslationX = scaledTranslationR * cos(translationTheta);

  double scaledRotation;
  if (rawRotation >= 0) {
    scaledRotation = pow(rawRotation, rotationScaling);
  } else {
    scaledRotation = std::copysign(pow(abs(rawRotation), rotationScaling), rawRotation);
  }

  // Apply joystick rate limits and calculate speed
  auto forwardSpeed = _yStickLimiter.Calculate(scaledTranslationY) * maxVelocity;
  auto sidewaysSpeed = _xStickLimiter.Calculate(scaledTranslationX) * maxVelocity;
  auto rotationSpeed = _rotStickLimiter.Calculate(scaledRotation) * maxAngularVelocity;

  // Dashboard things
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationY", rawTranslationY);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationX", rawTranslationX);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationR", rawTranslationR);
  frc::SmartDashboard::PutNumber(
      "Drivebase/Joystick Scaling/translationTheta (degrees)",
      translationTheta *
          (180 / math::pi));  // Multiply by 180/pi to convert radians to degrees
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationR",
                                 scaledTranslationR);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationY",
                                 scaledTranslationY);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationX",
                                 scaledTranslationX);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawRotation", rawRotation);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledRotation", scaledRotation);

  return frc::ChassisSpeeds{forwardSpeed * 1_mps, sidewaysSpeed * 1_mps, rotationSpeed * 1_tps};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller) {
  return Drive([this, &controller] { return CalcJoystickSpeeds(controller); }, true);
}

frc2::CommandPtr SubDrivebase::JoystickDriveSlow(frc2::CommandXboxController& controller) {
  return Drive([this, &controller] {
    auto speeds = CalcJoystickSpeeds(controller);
    speeds.vx = std::clamp(speeds.vx, -2.5_mps, 2.5_mps);
    speeds.vy = std::clamp(speeds.vy, -2.5_mps, 2.5_mps);
    return frc::ChassisSpeeds{speeds.vx, speeds.vy, speeds.omega};
  }, true);
}

frc2::CommandPtr SubDrivebase::RobotCentricDrive(frc2::CommandXboxController& controller) {
  return {SubDrivebase::GetInstance().Drive(
      [this, &controller] {
        auto speeds = CalcJoystickSpeeds(controller);
        std::swap(speeds.vx, speeds.vy);

        return speeds;
      }, false)};
}

frc2::CommandPtr SubDrivebase::Drive(std::function<frc::ChassisSpeeds()> speeds,
                                     bool fieldOriented) {
  return Run([this, speeds, fieldOriented] {
           auto speedVals = speeds();
           Drive(speedVals.vx, speedVals.vy, speedVals.omega, fieldOriented);
         })
      .FinallyDo([this] { Drive(0_mps, 0_mps, 0_deg_per_s, false); });
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                         units::turns_per_second_t rot, bool fieldRelative,
                         std::optional<std::array<units::newton_t, 4>> xForceFeedforwards,
                         std::optional<std::array<units::newton_t, 4>> yForceFeedforwards) {
  // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                    ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetGyroAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  constexpr bool inSim = frc::RobotBase::IsSimulation();
  speeds = frc::ChassisSpeeds::Discretize(speeds, inSim ? 20_ms : 60_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(
      &states,
      frc::SmartDashboard::GetNumber("Drivebase/Config/Max Velocity", MAX_VELOCITY.value()) *
          1_mps);

  // Extract force feedforwards
  std::array<units::newton_t, 4> defaults{0_N, 0_N, 0_N, 0_N};
  auto [flXForce, frXForce, blXForce, brXForce] = xForceFeedforwards.value_or(defaults);
  auto [flYForce, frYForce, blYForce, brYForce] = yForceFeedforwards.value_or(defaults);

  // Setting modules from aquired states
  auto [fl, fr, bl, br] = states;
  _frontLeft.SetDesiredState(fl, flXForce, flYForce);
  _frontRight.SetDesiredState(fr, frXForce, frYForce);
  _backLeft.SetDesiredState(bl, blXForce, blYForce);
  _backRight.SetDesiredState(br, brXForce, brYForce);
}

frc::ChassisSpeeds SubDrivebase::GetRobotRelativeSpeeds() {
  auto fl = _frontLeft.GetState();
  auto fr = _frontRight.GetState();
  auto bl = _backLeft.GetState();
  auto br = _backRight.GetState();
  return _kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();

  // config turn motors so it can run in auto init also. Had issues with parameters not being set on
  // startup
  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

frc2::CommandPtr SubDrivebase::SyncSensorBut() {
  return RunOnce([this] { SyncSensors(); });
}

frc::Rotation2d SubDrivebase::GetHeading() {
  return _poseEstimator.GetEstimatedPosition().Rotation();
}

frc::Rotation2d SubDrivebase::GetGyroAngle() {
  return _gyro.GetRotation2d();
}

frc::Rotation2d SubDrivebase::GetAllianceRelativeGyroAngle() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    return _gyro.GetRotation2d();
  } else {
    return _gyro.GetRotation2d() - 180_deg;
  }
}

units::meters_per_second_t SubDrivebase::GetVelocity() {
  // Use pythag to find velocity from x and y components
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                            _backLeft.GetState(), _backRight.GetState());
  namespace m = units::math;
  return m::sqrt(m::pow<2>(speeds.vx) + m::pow<2>(speeds.vy));
}

frc::SwerveDriveKinematics<4> SubDrivebase::GetKinematics() {
  return _kinematics;
}

// calculates the relative field location
void SubDrivebase::UpdateOdometry() {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    _poseEstimator.Update(GetGyroAngle(), {fl, fr, bl, br});
  } else {
    _poseEstimator.Update(GetGyroAngle() - 180_deg, {fl, fr, bl, br});
  }

  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

frc::ChassisSpeeds SubDrivebase::CalcDriveToPoseSpeeds(frc::Pose2d targetPose) {
  // Find current and target values
  DisplayPose("WERTY/targetPose", targetPose);
  double targetXMeters = targetPose.X().value();
  double targetYMeters = targetPose.Y().value();
  units::turn_t targetRotation = targetPose.Rotation().Radians();
  frc::Pose2d currentPosition = GetPose();
  double currentXMeters = currentPosition.X().value();
  double currentYMeters = currentPosition.Y().value();
  units::turn_t currentRotation = currentPosition.Rotation().Radians();

  // Use PID controllers to calculate speeds
  auto xSpeed = _teleopTranslationController.Calculate(currentXMeters, targetXMeters) * 1_mps;
  auto ySpeed = _teleopTranslationController.Calculate(currentYMeters, targetYMeters) * 1_mps;
  auto rSpeed = CalcRotateSpeed(currentRotation - targetRotation);

  // Clamp to max velocity
  xSpeed = units::math::min(xSpeed, MAX_DRIVE_TO_POSE_VELOCITY);  // Max_Velocity
  xSpeed = units::math::max(xSpeed, -MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::min(ySpeed, MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::max(ySpeed, -MAX_DRIVE_TO_POSE_VELOCITY);

  frc::SmartDashboard::PutNumber("CalcDriveLogs/xSpeed", -xSpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/ySpeed", ySpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/rSpeed", rSpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/targetXMeters", targetXMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/targetYMeters", targetYMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentXMeters", currentXMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentYMeters", currentYMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentRotation", currentRotation.value());
  return frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed};
}

units::turns_per_second_t SubDrivebase::CalcRotateSpeed(units::turn_t rotationError) {
  auto omega = _teleopRotationController.Calculate(rotationError, 0_deg) * 1_rad_per_s;
  omega = units::math::min(omega, MAX_ANGULAR_VELOCITY);
  omega = units::math::max(omega, -MAX_ANGULAR_VELOCITY);
  return omega;
}

bool SubDrivebase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = _poseEstimator.GetEstimatedPosition();
  auto rotError = currentPose.Rotation() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());
  auto velocity = GetVelocity();
  DisplayPose("current pose", currentPose);
  DisplayPose("target pose", pose);

  frc::SmartDashboard::PutNumber("Drivebase/rotError",
                                 units::math::abs(rotError.Degrees()).value());
  frc::SmartDashboard::PutNumber("Drivebase/posError", posError.value());

  frc::SmartDashboard::PutBoolean("Drivebase/IsAtPose",
                                  units::math::abs(rotError.Degrees()) < 1_deg && posError < 2_cm);

  if (units::math::abs(rotError.Degrees()) < 1_deg && posError < 2_cm && velocity < 0.0001_mps) {
    return true;
  } else {
    return false;
  }
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.ZeroYaw();
}

frc2::CommandPtr SubDrivebase::ResetGyroCmd() {
  return RunOnce([this] { ResetGyroHeading(0_deg); });
}

frc::Pose2d SubDrivebase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SubDrivebase::GetSimPose() {
  return _simPoseEstimator.GetEstimatedPosition();
}

void SubDrivebase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}



void SubDrivebase::AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev) {
  _poseEstimator.AddVisionMeasurement(frc::Pose2d{pose.X(), pose.Y(), pose.Rotation()}, timeStamp, dev);
}

void SubDrivebase::SetBrakeMode(bool mode) {
  _frontLeft.SetBreakMode(mode);
  _frontRight.SetBreakMode(mode);
  _backLeft.SetBreakMode(mode);
  _backRight.SetBreakMode(mode);
}

