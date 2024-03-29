#pragma once
#include <string>


#include "LEDController.h"
#include "utility.h"
#include "SwerveDrive.h"

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/Preferences.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorMatch.h>
#include <rev/ColorSensorV3.h>
#include <Toggle.h>


#include <frc/controller/RamseteController.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>
//#include <frc2/command/SubsystemBase.h>
#include <units/units.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <AHRS.h> // navx
// #include <frc/cs/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void HandleColorWheel();
  void HandleDrivetrain();
  void HandleLEDStrip();
  void HandleVision();
  void HandleShooter();
  void HandleIntake();
  void AutonIntakeAndShoot(std::string trenchPath, std::string shootPath);
  void HandleElevator();
  /*=============
  Pins & IDs
  =============*/
  const int FRWheelID = 12;
  const int FLWheelID = 13;
  const int BRWheelID = 14;
  const int BLWheelID = 15;

  const int FRTurnID = 19;
  const int FLTurnID = 20;
  const int BRTurnID = 21;
  const int BLTurnID = 22;

  const int FRCANCoderID = 23;
  const int FLCANCoderID = 21;
  const int BRCANCoderID = 20;
  const int BLCANCoderID = 22;

  const int ColorWheelID = 16;
  const int FlyWheelID = 10;
  const int FollowFlywheelID = 11;
  const int ElevatorID = 18;
  const int solenoidID = 0;
  const int colorwheelPistonID = 1;
  const int intakeMotorID = 5;
  const int beltID = 7;
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  static constexpr auto MXPPort = frc::SPI::Port::kMXP;
  /*=============
  Driverstation
  =============*/

  nt::NetworkTableInstance networkTableInstance = nt::NetworkTableInstance::GetDefault();
  frc::Preferences& prefs = *frc::Preferences::GetInstance();
  std::shared_ptr<nt::NetworkTable> debugTab = networkTableInstance.GetTable("Shuffleboard")->GetSubTable("debug");
	frc::SendableChooser<std::string> autonChooser;
  std::string noneAuton = "Nothing";
  std::string simpleAuton = "Simple";
  std::string basicAuton = "Basic";
  std::string middlePathAuton = "Middle Path Auton";
  std::string straightPathAuton = "Straight Path Auton";
  std::string specialAuton = "Special Path Auton";
  std::shared_ptr<nt::NetworkTable> visionTab = networkTableInstance.GetTable("Shuffleboard")->GetSubTable("vision");

  /*=============
  Color Sensor
  =============*/
  rev::ColorSensorV3 colorSensor{i2cPort};
  TalonSRX colorwheelMotor{ColorWheelID};	
  char currentColorTarget = 'N';
  frc::Solenoid colorwheelPiston{colorwheelPistonID};

  /*=============
  Drivetrain
  =============*/
  std::tuple<int, int, int> fl_id{FLWheelID, FLTurnID, FLCANCoderID};
  std::tuple<int, int, int> fr_id{FRWheelID, FRTurnID, FRCANCoderID};
  std::tuple<int, int, int> br_id{BRWheelID, BRTurnID, BRCANCoderID};
  std::tuple<int, int, int> bl_id{BLWheelID, BLTurnID, BLCANCoderID};

  const double robot_width_in = 20.0;
  const double robot_length_in = 20.0;
  SwerveDrive swerve_drive{fl_id, fr_id, bl_id, br_id, robot_width_in, robot_length_in};
  /*=============
  Controls
  =============*/
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;
  frc::XboxController pilot{0};
  frc::XboxController copilot{1};

  /*=============
  flywheel, intake, belts
  =============*/
  static constexpr double kTalonRPMConversionFactor = 10.0 / 2048.0 * 60.0; // 100ms, 2048 ticks
  static const int beltFeedTicks = 5000;
  int beltFireTicks = 9000;
  double flywheelRPM = 4000;
  static constexpr double flywheelReverseRPM = 900;
  static constexpr double flywheelStoppedRPM = 10;
  static constexpr double intakeRollerSpeed = 0.6;
  static constexpr double reverseBeltSpeed = 0.85;
  static constexpr double forwardBeltSpeed = 0.7;
  static constexpr double colorSpinnerSpeed = 0.5;
  TalonFX flywheelMotor{FlyWheelID};
  TalonFX flywheelMotorFollow{FollowFlywheelID};
  frc::Solenoid intakePiston{solenoidID};
  TalonSRX intakeMotor{intakeMotorID};
  TalonSRX belt{beltID}; // vertical + bottom
  frc::DigitalInput lineBreak1{0};
  frc::DigitalInput lineBreak2{1};
  frc::DigitalInput lineBreak3{2};

	bool isUpToSpeed = false;

  /*=============
  autonomous
  =============*/
  frc::Timer SimpleTimeFromStart;
  static constexpr auto kMaxSpeed = 1_mps; 
  static constexpr auto kMaxAcceleration = 1_mps_sq;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  static constexpr double kRamseteB = 2;     
  static constexpr double kRamseteZeta = 0.7;
  // https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html
  AHRS gyro = AHRS{MXPPort};

  int stage = 0;
  bool timerStarted = false;
  frc::Timer BasicTimer;
  static constexpr double centerCamera = 80.0;
  /*=============
  vision
  =============*/
  bool frontCamera = true;
  bool isAutoAligning = false;
  /*=============
  LED Strip
  =============*/
  LEDController ledStrip{36, 9};
  int ledMode = 0;
  Toggle ledUp;
  Toggle ledDown;
  /*=============
  Elevator
  =============*/
  int elevatorBreaksPoint = 600000;
  int maxElevatorUp = 300000;
  int minElevatorDown = 250000;
  double elevatorSpeedUp = 0.75;
  double elevatorSpeedDown = 0.6;
  TalonFX elevatorMotor{ElevatorID};
  double inputScale = 1.0;
  /*===========
  Counting balls
  */
  Toggle LinebreakIn{false};
  Toggle LinebreakOut{false};
  bool ignoreCountingOut = true;
  bool ignoreCountingIn = true;
  int count = 0;
};