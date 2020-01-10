#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/GenericHID.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <persistent.h>
#include <SparkController.h>
#include <frc/Preferences.h>

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
  void InitializePIDController(rev::CANPIDController);
  // define pin numbers for motors
  const int RLeadID = 1;
  const int LLeadID = 2;
  const int RFollowID = 3;
  const int LFollowID = 4;


  rev::CANSparkMax RLeadMotor{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollowMotor{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLeadMotor{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollowMotor{LFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController LLeadPID{LLeadMotor};
  rev::CANPIDController RLeadPID{RLeadMotor};

  SparkController RLead{RLeadMotor, RLeadPID};
  SparkController LLead{LLeadMotor, LLeadPID};
  frc::DifferentialDrive drivetrain{LLead, RLead};

  //create controls
  frc::XboxController pilot{0};
  frc::XboxController copilot{1};
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;
  frc::Preferences& prefs = *frc::Preferences::GetInstance();
// http://www.revrobotics.com/sparkmax-users-manual/

};
