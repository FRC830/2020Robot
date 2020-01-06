#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
void Robot::RobotInit() {
    // clear the motor config
    left_lead_motor.RestoreFactoryDefaults();
    left_follow_motor.RestoreFactoryDefaults();
    right_lead_motor.RestoreFactoryDefaults();
    right_follow_motor.RestoreFactoryDefaults();
    // Follow motors
    left_follow_motor.Follow(left_lead_motor);
    right_follow_motor.Follow(right_lead_motor);
    
    persistent P = persistent("p", 1);

    // rev::CANPIDController left_lead_controller = CreatePIDController(left_follow_motor);
    // rev::CANPIDController right_lead_controller = CreatePIDController(left_follow_motor);
}
rev::CANPIDController Robot::CreatePIDController(rev::CANSparkMax motor) {
    
    // return a configured PID Controller
  rev::CANPIDController pid_controller = motor.GetPIDController();
  pid_controller.SetP(1);
  pid_controller.SetI(1);
  pid_controller.SetD(1);
  return pid_controller;
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  
}

void Robot::AutonomousPeriodic() {
 
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif