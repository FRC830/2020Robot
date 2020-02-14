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

#include "ctre/Phoenix.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "LEDController.h"
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <Toggle.h>
#include <fstream>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>



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
  // double ProcessControllerInput(double);
  void HandleColorWheel();
  void HandleDrivetrain();
  void HandleLEDStrip();
  void HandleVision();
  void HandleStuff();
  void HandleRecordPlayback();



  //when we reset the motors there are some reidual values. Therefore, we want to ignore the first two durring playback.
  int runsAfterPlayback = 5;

  void print(std::vector<std::vector<double>> input);
  void printSD(std::vector<double> input, std::string name);
  
  std::tuple<char, double> ClosestColor();
  void MakeSlider(std::string, double, double=255);
  void InitializePIDController(rev::CANPIDController);
  void HandleShooter();
  void HandleIntake();
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
  bool recordGo = false;

  //defines motors and PID controllers
  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollowMotor{LFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController LLeadPID{LLeadMotor};
  rev::CANPIDController RLeadPID{RLeadMotor};
  rev::CANPIDController LFollowPID{LLeadMotor};
  rev::CANPIDController RFollowPID{RLeadMotor};
  //defines drivestrain and motor controllers
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

  TalonSRX colorWheelMotor{ColorWheelID};

  char currentColorTarget = 'N';
  

  TalonFX flywheelMotor{FlyWheelID};

  //solenoid id
  const int solenoidID = 0;
  const int intakeMotorID = 5;
  const int shooterID = 6;
  const int intakeBeltID = 7;
  frc::Solenoid intakePiston{solenoidID};
  // Toggle canIntake{false};
  // Toggle isShooting{false};
  VictorSPX intakeMotor{intakeMotorID};
  TalonSRX intakeBelt{intakeBeltID}; // vertical + bottom
  VictorSPX shooterBelt{shooterID};// top belt
  frc::DigitalInput lineBreak1{0};
  frc::DigitalInput lineBreak2{1};
  double intakeBeltSpeed = 0;
  double shooterBeltSpeed = 0.2;
	bool isUpToSpeed = false;

  //Reversing and Counting the balls
  //Linebreak Sensor 3 has a value of 1 when both sensors are not facing eachother
  //Linebreak Sensor 3 has a value of 0 when both sensors are facing eachother
  //Linebreak sensors can be displaced by about 9.5 inches from each other and about up to 1 cm of vertical displacement
  frc::DigitalInput lineBreak3{2};
  int ballsStored = 0;
  int ballsShot = 0;

  bool lineBreak1WasBroken = false;
  bool lineBreak2WasBroken = false;
  bool lineBreak3WasBroken = false;

  //int runsAfterPlayback = 5;

  double kPposi = 0.17, kIposi = 1e-3, kDposi = 0;

  std::vector<double> leftLeadMotorValues;
std::vector<double> rightLeadMotorValues;
std::vector<double> leftFollowMotorValues;
std::vector<double> rightFollowMotorValues;
};
