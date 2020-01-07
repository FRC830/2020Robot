/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

  //rev::CANSparkMax	(	int 	deviceID,MotorType 	type )	
  // create motors
  const int RLeadID = 1;
  const int LLeadID = 2;
  const int RFollowID = 3;
  const int LFollowID = 4;
  rev::CANSparkMax RLead{RLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RFollow{LLeadID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LLead{RFollowID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax LFollow{LFollowID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANPIDController LLeadPID{RLead};
  rev::CANPIDController RLeadPID{RLead};

  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp
  SparkController RLeadMotor{RLead, RLeadPID};
  SparkController LLeadMotor{LLead, LLeadPID};


  //create controls
  frc::XboxController pilot{0};
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand; 
  static constexpr double DEADZONE_THRESHOLD = 0.1;
  persistent<double> Deadzone{"deadzone",0.1};
  persistent<double> P{"P",1};
  persistent<double> I{"I",0};
  persistent<double> D{"D",0};
  // http://www.revrobotics.com/sparkmax-users-manual/
  persistent<int> MaxRPM{"maxRPM",2400};
};
