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
  const int right_lead_id = 1;
  const int left_lead_id = 2;
  const int right_follow_id = 3;
  const int left_follow_id = 4;
  rev::CANSparkMax right_lead_motor{right_lead_id, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_follow_motor{right_follow_id, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax left_lead_motor{left_lead_id, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax left_follow_motor{left_follow_id, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANPIDController left_lead_controller{left_lead_motor};
  rev::CANPIDController right_lead_controller{right_lead_motor};
  rev::CANEncoder left_lead_encoder{left_lead_motor};
  rev::CANEncoder right_lead_encoder{right_lead_motor};
  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp

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
