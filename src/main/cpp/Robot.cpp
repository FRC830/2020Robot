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
    
    
    
}
void Robot::CreatePIDController(motor rev::CANSparkMax) {

    // return a configured PID Controller
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  
}

void Robot::AutonomousPeriodic() {
 
}

void Robot::TeleopInit() {
    
}

void Robot::TeleopPeriodic() {
    pilot.ArcadeDrive(-)
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif