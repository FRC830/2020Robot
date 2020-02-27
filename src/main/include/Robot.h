  #pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/GenericHID.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <SparkController.h>
#include <frc/Preferences.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "LEDController.h"
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <Toggle.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "utility.h"


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
  void HandleRecordPlayback();
  void HandleColorWheel();
  void HandleDrivetrain();
  void HandleLEDStrip();
  void HandleVision();
  void HandleShooter();
  void HandleIntake();
  void HandlePathweaver();
  void HandleElevator();
  // define pin numbers for motors
  const int RLeadID = 2;
  const int LLeadID = 4;
  const int RFollowID = 1;
  const int LFollowID = 3;
  const int ColorWheelID = 16;
  const int FlyWheelID = 17;
  const int ElevatorID = 18;

  //colors	
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;	
  rev::ColorSensorV3 colorSensor{i2cPort};	

  TalonSRX colorWheelMotor{ColorWheelID};	

  char currentColorTarget = 'N';

  //defines motors and PID controllers
  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollowMotor{LFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController LLeadPID{LLeadMotor};
  rev::CANPIDController RLeadPID{RLeadMotor};
  rev::CANPIDController LFollowPID{LFollowMotor};
  rev::CANPIDController RFollowPID{RFollowMotor};
  //defines drivestrain and motor controllers
  SparkController RLead{RLeadMotor, RLeadPID};
  SparkController LLead{LLeadMotor, LLeadPID};
  SparkController RFollow{RFollowMotor, RFollowPID};
  SparkController LFollow{LFollowMotor, LFollowPID};
  frc::DifferentialDrive drivetrain{LLead, RLead};

  //create controls
  frc::XboxController pilot{0};
  frc::XboxController copilot{1};
  //frc::GenericHID::kLeftHand to LEFT, etc
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;

  //initializes prefreences widget
  nt::NetworkTableInstance networkTableInstance = nt::NetworkTableInstance::GetDefault();
  frc::Preferences& prefs = *frc::Preferences::GetInstance();
  //Constant Values
  int flywheelSpeedVelocity = 12100;
  static const int intakeBeltSpeedVelocity = 5000;
  int intakeBeltShootVelocity = 12000;
  static const int flywheelReverseVelocity = -1000;
  static const int flywheelStoppedVelocity = 50;
  static constexpr double intakeRollerSpeed = 0.8;
  static constexpr double reverseBeltSpeed = 1;
  static constexpr double forwardBeltSpeed = 1;
  static constexpr double colorSpinnerSpeed = 0.5;
  // LED
  LEDController ledStrip{40, 9};
  int ledMode = 0;
// http://www.revrobotics.com/sparkmax-users-manual/

  TalonFX flywheelMotor{FlyWheelID};

  //solenoid id 
  const int solenoidID = 0;
  const int intakeMotorID = 5;
  // const int shooterID = 6;
  const int intakeBeltID = 7;
  frc::Solenoid intakePiston{solenoidID};
  // Toggle canIntake{false};
  // Toggle isShooting{false};
  TalonSRX intakeMotor{intakeMotorID};
  TalonSRX intakeBelt{intakeBeltID}; // vertical + bottom
  //VictorSPX shooterBelt{shooterID};// top belt
  frc::DigitalInput lineBreak1{0};
  frc::DigitalInput lineBreak2{1};
	bool isUpToSpeed = false;

  //Reversing and Counting the balls
  //Linebreak Sensor 3 has a value of 1 when both sensors are not facing eachother
  //Linebreak Sensor 3 has a value of 0 when both sensors are facing eachother
  //Linebreak sensors can be displaced by about 9.5 inches from each other and about up to 1 cm of vertical displacement
  frc::DigitalInput lineBreak3{2};

  // Robot characterization
  static constexpr auto ks = 0.167;
  static constexpr auto kv = 0.0684; // TODO convert to seconds-per-meter
  // https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/entering-constants.html
  static constexpr auto ka = 0.00744; // TODO convert to seconds^2-per-meter
  // test value developed from analyzing characterization
  static constexpr double kPDriveVel = 0.339;

  static constexpr units::inch_t kTrackwidth = 27.9_in;
  frc::DifferentialDriveKinematics kDriveKinematics{units::meter_t(kTrackwidth)};

  static constexpr auto kMaxSpeed = 1_mps; 
  static constexpr auto kMaxAcceleration = 1_mps_sq;

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds

  static constexpr double kRamseteB = 2;     
  static constexpr double kRamseteZeta = 0.7;
  // https://docs.wpilib.org/en/latest/docs/software/examples-tutorials/trajectory-tutorial/creating-drive-subsystem.html
  frc::ADXRS450_Gyro gyro;
  frc::DifferentialDriveOdometry odometry{units::radian_t(0)};
	frc::SendableChooser<std::string> autonChooser;
  std::string defaultAuton = "Nothing";
  std::string simpleAuton = "Simple";
  std::string pathAuton = "Path";
  frc::RamseteController controller;

  frc::Trajectory trajectory;

  frc::Timer TimeFromStart;

  //vision
  bool frontCamera = true;
  std::shared_ptr<nt::NetworkTable> visionTab2 = networkTableInstance.GetTable("Shuffleboard")->GetSubTable("vision");

  //playback and record

  //when we reset the motors there are some reidual values. Therefore, we want to ignore the first two durring playback.
  size_t runsAfterPlayback = 5; // avoid warnings

  bool isRecording = false;
  bool PlayingBack = false;
  bool Adown = false;
  bool recordGo = false;

  double targetVelocity;
  bool isAutoAligning = false;
  
  std::vector<double> leftLeadMotorValues {};
std::vector<double> rightLeadMotorValues;
std::vector<double> leftFollowMotorValues;
std::vector<double> rightFollowMotorValues;

double kPposi = 0.17, kIposi = 1e-3, kDposi = 0;
  Toggle ledUp;
  Toggle ledDown;
  static constexpr double centerCamera = 80.0;

  bool reversedpath = false;


  //Elevator
  int goingUp = 300000;
  int goingDown = 350000;
  double elevatorSpeed = 0.3;
  TalonFX elevatorMotor{ElevatorID};

  //double scaleFactor = 
};