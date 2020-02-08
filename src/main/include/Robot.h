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
  void HandleShooter();
  // define pin numbers for motors
  const int RLeadID = 2;
  const int LLeadID = 4;
  const int RFollowID = 1;
  const int LFollowID = 3;
  const int ColorWheelID = 16;
  const int FlyWheelID = 17;

  //defines motors and PID controllers
  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollowMotor{LFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController LLeadPID{LLeadMotor};
  rev::CANPIDController RLeadPID{RLeadMotor};
  //defines drivestrain and motor controllers
  SparkController RLead{RLeadMotor, RLeadPID};
  SparkController LLead{LLeadMotor, LLeadPID};
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
  // frc::Solenoid intakePiston{solenoidID};
  // Toggle canIntake{false};
  // Toggle isShooting{false};
  VictorSPX intakeMotor{intakeMotorID};
  TalonSRX intakeBelt{intakeBeltID}; // vertical + bottom
  VictorSPX shooterBelt{shooterID};// top belt
  frc::DigitalInput lineBreak{0};
  frc::DigitalInput lineBreak2{1};
  double intakeBeltSpeed = 0;
  double shooterBeltSpeed = 0.2;
	bool isUpToSpeed = false;

};