#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
void Robot::RobotInit() {
    // clear the motor config
    LLeadMotor.RestoreFactoryDefaults();
    LFollowMotor.RestoreFactoryDefaults();
    RLeadMotor.RestoreFactoryDefaults();
    RFollowMotor.RestoreFactoryDefaults();
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp
    // basic initialization
    InitializePIDController(LLeadPID);
    InitializePIDController(RLeadPID);
    LFollowMotor.Follow(LLeadMotor);
    RFollowMotor.Follow(RLeadMotor);
}
// return a configured PID Controller
void Robot::InitializePIDController(rev::CANPIDController pid_controller) {
    
    // default smart motion coefficients
  double kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
  pid_controller.SetOutputRange(-1, 1);
  pid_controller.SetSmartMotionMinOutputVelocity(kMinVel);
  pid_controller.SetSmartMotionMaxAccel(kMaxAcc);
  pid_controller.SetSmartMotionAllowedClosedLoopError(kAllErr);
}
void Robot::UpdatePIDController(rev::CANPIDController pidController) {
    if (MaxRPM.justUpdated()) { pidController.SetSmartMotionMaxVelocity(MaxRPM.get()); }
    if (P.justUpdated()) { pidController.SetP(P.get()); }
    if (I.justUpdated()) { pidController.SetI(I.get()); }
    if (D.justUpdated()) { pidController.SetD(D.get()); }
}
double Robot::ProcessControllerInput(double val) {
  return fabs(val) < Deadzone.get() ? 0 : val;
}
void Robot::RobotPeriodic() {
  UpdatePIDController(RLeadPID);
  UpdatePIDController(LLeadPID);
  double speed = -ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(LEFT));
  double targetVelocity = speed * MaxRPM.get();
  frc::SmartDashboard::PutNumber("Speed", speed);
  frc::SmartDashboard::PutNumber("Turn", turn);

  drivetrain.ArcadeDrive(targetVelocity, turn, true);

  frc::SmartDashboard::PutNumber("Left Lead Encoder Position", LLead.GetEncoder());
  frc::SmartDashboard::PutNumber("Left Lead Output", LLead.GetApplied());
  frc::SmartDashboard::PutNumber("Right Lead Encoder Position", RLead.GetEncoder());
  frc::SmartDashboard::PutNumber("Right Lead Output", RLead.GetApplied());
}


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