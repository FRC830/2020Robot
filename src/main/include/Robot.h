#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/GenericHID.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <SparkController.h>
#include <frc/Preferences.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/util/Color.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "LEDController.h"
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <Toggle.h>

#include <fstream>


// #include <frc/cs/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  double ProcessControllerInput(double);
  void HandleColorWheel();
  void HandleDrivetrain();
  void HandleLEDStrip();
  void HandleVision();
  void HandleStuff();

  int runsAfterPlayback = 0;

  void print(std::vector<double> input);
  void printSD(std::vector<double> input, std::string name);
  
  std::tuple<char, double> ClosestColor();
  void MakeSlider(std::string, double, double=255);
  void InitializePIDController(rev::CANPIDController);
  // define pin numbers for motors
  const int RLeadID = 2;
  const int LLeadID = 4;
  const int RFollowID = 1;
  const int LFollowID = 3;
  const int ColorWheelID = 16;
  const int FlyWheelID = 17;
  bool isRecording = false;
  bool PlayingBack = false;
  bool Adown = false;
  //defines motors and PID controllers
  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollowMotor{LFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController LLeadPID{LLeadMotor};
  rev::CANPIDController RLeadPID{RLeadMotor};
  rev::CANPIDController LFollowPID{LFollowMotor};
  rev::CANPIDController RFollowPID{RFollowMotor};
  // intake

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
  frc::Preferences& prefs = *frc::Preferences::GetInstance();

  // LED
  LEDController ledStrip{40, 9};
  int ledMode = 0;
// http://www.revrobotics.com/sparkmax-users-manual/

  //colors
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};
  rev::ColorMatch colorMatcher;

  TalonSRX colorWheelMotor{ColorWheelID};
  frc::Color aimRed = {0.465, 0.376, 0.158}; 
  frc::Color aimYellow = {0.324, 0.535, 0.14}; 
  frc::Color aimGreen = {0.197, 0.545, 0.256}; 
  frc::Color aimBlue = {0.157, 0.43, 0.412}; 
  char currentColorTarget = 'N';
  

  TalonFX flywheelMotor{FlyWheelID};

  //solenoid id
  const int solenoidID = 0;
  const int intakeMotorID = -1;
  const int shooterID = -1;
  const int intakeBeltID = -1;
//  frc::Solenoid intakePiston{solenoidID};
  Toggle canIntake{false};
  Toggle isShooting{false};
  VictorSPX intakeMotor{intakeMotorID};
  VictorSPX intakeBelt{intakeBeltID}; // vertical + bottom
  VictorSPX shooterBelt{shooterID};

  std::vector<double> leftLeadMotorValues;
  std::vector<double> rightLeadMotorValues;
  std::vector<double> leftFollowMotorValues;
  std::vector<double> rightFollowMotorValues;

  //we need to add velocity PID as well
  double kPposi = 0.1, kIposi = 1e-4, kDposi = 1;
};