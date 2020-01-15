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
// #include <ctre/TalonFX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "LEDController.h"


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
  std::tuple<std::string, double> ClosestColor();
  void MakeSlider(std::string, double, double=255);
  void InitializePIDController(rev::CANPIDController);
  // define pin numbers for motors
  const int RLeadID = 1;
  const int LLeadID = 2;
  const int RFollowID = 3;
  const int LFollowID = 4;
  const int ColorWheelID = 16;
  //defines motors and PID controllers
  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
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
  //abrieveate frc::GenericHID::kLeftHand to LEFT, etc
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;

  //initializes prefreences widget
  frc::Preferences& prefs = *frc::Preferences::GetInstance();


  LEDController ledStrip{40, 9};
// http://www.revrobotics.com/sparkmax-users-manual/

  //colors
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};
  rev::ColorMatch colorMatcher;
  int ledMode = 0;

  TalonSRX colorWheelMotor{ColorWheelID};
  frc::Color aimRed = {0.465, 0.376, 0.158}; 
  frc::Color aimYellow = {0.324, 0.535, 0.14}; 
  frc::Color aimGreen = {0.197, 0.545, 0.256}; 
  frc::Color aimBlue = {0.157, 0.43, 0.412}; 


  //TalonFX Code

  const int TalonDeviceID = 17;

  TalonFX falcon{TalonDeviceID};



  
  /*
  in robotPeriodic
  
  max velocity = preferences.GetNumber("maxV")
  falcon.Set(controller input * Max velocity, ctre::longnamespacething::velocity mode)
  http://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html#aa3e3514f29187deaa9d843592f42392b
  */
};


  