#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/filter/SlewRateLimiter.h>
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include "utilities/SwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DigitalInput.h>
#include "studica/AHRS.h"

class SubDrivebase : public frc2::SubsystemBase {
 public:
  SubDrivebase();
  static SubDrivebase& GetInstance() {
    static SubDrivebase inst;
    return inst;
  }
  void Periodic() override;
  void SimulationPeriodic() override;

  // Instantaneous functions
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev);
  void ResetGyroHeading(units::degree_t startingAngle = 0_deg);
  void SetBrakeMode(bool mode);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void UpdateOdometry();
  void SyncSensors();

  // Getters
  bool IsAtPose(frc::Pose2d pose);
  
  frc::ChassisSpeeds CalcDriveToPoseSpeeds(frc::Pose2d targetPose);
  frc::ChassisSpeeds CalcJoystickSpeeds(frc2::CommandXboxController& controller);
  units::turns_per_second_t CalcRotateSpeed(units::turn_t rotationError);
  frc::Pose2d GetPose();
  frc::Pose2d GetSimPose();
  frc::Rotation2d GetHeading();  // Heading as recorded by the pose estimator (matches field orientation)
  frc::Rotation2d GetGyroAngle();  // Heading as recorded by the gyro (zero is direction when switched on)
  frc::Rotation2d GetAllianceRelativeGyroAngle();
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  // Commands
  frc2::CommandPtr GyroCoralLeftStationAlign(frc2::CommandXboxController& controller);
  frc2::CommandPtr GyroCoralRightStationAlign(frc2::CommandXboxController& controller);
  frc2::CommandPtr JoystickDrive(frc2::CommandXboxController& controller);
  frc2::CommandPtr JoystickDriveSlow(frc2::CommandXboxController& controller);
  frc2::CommandPtr WheelCharecterisationCmd();
  frc2::CommandPtr Drive(std::function<frc::ChassisSpeeds()> speeds, bool fieldOriented);
  frc2::CommandPtr RobotCentricDrive(frc2::CommandXboxController& controller);
  void DriveToPose(frc::Pose2d targetPose);
  frc2::CommandPtr SyncSensorBut();
  frc2::CommandPtr ResetGyroCmd();
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Dynamic(direction);
  }

  // Constants
  static constexpr units::meters_per_second_t MAX_VELOCITY = 5_mps;
  static constexpr units::meters_per_second_t MAX_DRIVE_TO_POSE_VELOCITY = 1_mps;
  static constexpr units::turns_per_second_t MAX_ANGULAR_VELOCITY =
      290_deg_per_s;  // CHANGE TO 720\[]

  static constexpr units::turns_per_second_squared_t MAX_ANG_ACCEL{std::numbers::pi};

  static constexpr double MAX_JOYSTICK_ACCEL = 5;
  static constexpr double MAX_ANGULAR_JOYSTICK_ACCEL = 3;
  static constexpr double JOYSTICK_DEADBAND = 0.08;
  static constexpr double TRANSLATION_SCALING = 2;  // Set to 1 for linear scaling
  static constexpr double ROTATION_SCALING = 1;     // Set to 1 for linear scaling

 private:
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
             units::turns_per_second_t rot, bool fieldRelative,
             std::optional<std::array<units::newton_t, 4>> xForceFeedforwards = std::nullopt,
             std::optional<std::array<units::newton_t, 4>> yForceFeedforwards = std::nullopt);

  studica::AHRS _gyro{studica::AHRS::NavXComType::kMXP_SPI};

  // Swerve modules
  frc::Translation2d _frontLeftLocation{+233.061165_mm, +233.061165_mm};
  frc::Translation2d _frontRightLocation{+233.061165_mm, -233.061165_mm};
  frc::Translation2d _backLeftLocation{-233.061165_mm, +233.061165_mm};
  frc::Translation2d _backRightLocation{-233.061165_mm, -233.061165_mm};

  const units::turn_t FRONT_RIGHT_MAG_OFFSET = //
       (0.008057+0.5)  * 1_tr;
  const units::turn_t FRONT_LEFT_MAG_OFFSET = //
       0.757568 * 1_tr;
  const units::turn_t BACK_RIGHT_MAG_OFFSET = //
       (-0.43896484375 + 0.5)  * 1_tr;
  const units::turn_t BACK_LEFT_MAG_OFFSET =
       -0.437255859375 * 1_tr;

  frc::DigitalInput _toggleBrakeCoast{dio::BRAKE_COAST_BUTTON};
  frc::DigitalInput _armZeroButton{dio::ARM_ZERO_BUTTON};

  SwerveModule _frontLeft{canid::DRIVEBASE_FRONT_LEFT_DRIVE, canid::DRIVEBASE_FRONT_LEFT_TURN,
                          canid::DRIVEBASE_FRONT_LEFT_ENCODER, (FRONT_LEFT_MAG_OFFSET)};
  SwerveModule _frontRight{canid::DRIVEBASE_FRONT_RIGHT_DRIVE, canid::DRIVEBASE_FRONT_RIGHT_TURN,
                           canid::DRIVEBASE_FRONT_RIGHT_ENCODER, (FRONT_RIGHT_MAG_OFFSET)};
  SwerveModule _backLeft{canid::DRIVEBASE_BACK_LEFT_DRIVE, canid::DRIVEBASE_BACK_LEFT_TURN,
                         canid::DRIVEBASE_BACK_LEFT_ENCODER, (BACK_LEFT_MAG_OFFSET)};
  SwerveModule _backRight{canid::DRIVEBASE_BACK_RIGHT_DRIVE, canid::DRIVEBASE_BACK_RIGHT_TURN,
                          canid::DRIVEBASE_BACK_RIGHT_ENCODER, (BACK_RIGHT_MAG_OFFSET)};

  // Control objects
  frc::SwerveDriveKinematics<4> _kinematics{_frontLeftLocation, _frontRightLocation,
                                            _backLeftLocation, _backRightLocation};

  frc::PIDController _teleopTranslationController{1.7, 0.0, 0.0};
  frc::ProfiledPIDController<units::radian> _teleopRotationController{
      3, 0, 0.2, {MAX_ANGULAR_VELOCITY, MAX_ANG_ACCEL}};

  // Pose estimation
  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      _kinematics,
      _gyro.GetRotation2d(),
      {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _frontRight.GetAngle()},
       frc::SwerveModulePosition{0_m, _backLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _backRight.GetAngle()}},
      frc::Pose2d()};

  frc::Field2d _fieldDisplay;

  // Sim pose estimation
  frc::SwerveDrivePoseEstimator<4> _simPoseEstimator{
      _kinematics,
      _gyro.GetRotation2d(),
      {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _frontRight.GetAngle()},
       frc::SwerveModulePosition{0_m, _backLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _backRight.GetAngle()}},
      frc::Pose2d()};

  // Joystick controller rate limiters
  double _tunedMaxJoystickAccel = MAX_JOYSTICK_ACCEL;
  double _tunedMaxAngularJoystickAccel = MAX_ANGULAR_JOYSTICK_ACCEL;
  frc::SlewRateLimiter<units::scalar> _xStickLimiter{_tunedMaxJoystickAccel / 1_s};
  frc::SlewRateLimiter<units::scalar> _yStickLimiter{_tunedMaxJoystickAccel / 1_s};
  frc::SlewRateLimiter<units::scalar> _rotStickLimiter{_tunedMaxAngularJoystickAccel / 1_s};

  // Sysid
  frc2::sysid::SysIdRoutine _sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            _frontLeft.DriveStraightVolts(driveVoltage);
            _backLeft.DriveStraightVolts(driveVoltage);
            _frontRight.DriveStraightVolts(driveVoltage);
            _backRight.DriveStraightVolts(driveVoltage);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-left")
                .voltage(_frontLeft.GetDriveVoltage())
                .position(_frontLeft.GetDrivenRotations().convert<units::turns>())
                .velocity(_frontLeft.GetSpeed());
            log->Motor("drive-right")
                .voltage(_frontRight.GetDriveVoltage())
                .position(_frontRight.GetDrivenRotations().convert<units::turns>())
                .velocity(_frontRight.GetSpeed());
          },
          this}};
};