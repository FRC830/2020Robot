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
#include <frc/DifferentialDrive.h>
#include <frc/GenericHID.h>
#include <frc/SpeedControllerGroup.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void CreatePIDController();
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
  
  //create drivetrain
  //frc::SpeedControllerGroup right{front_right_motor,back_right_motor};
  //frc::SpeedControllerGroup left{front_left_motor,back_left_motor};

 // frc::DifferentialDrive drivetrain{left, right};
 
  // frc::DifferentialDrive drivetrain{left_lead_motor,right_lead_motor};

  //create controls
  frc::XboxController pilot{0};
  static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
  static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand; 
  static constexpr double DEADZONE_THRESHOLD = 0.1;

};
