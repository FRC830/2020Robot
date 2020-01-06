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
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp

    InitializePIDController(left_lead_controller);
    InitializePIDController(right_lead_controller);

    left_lead_encoder = left_lead_motor.GetEncoder();
    right_lead_encoder = right_lead_motor.GetEncoder();
}
// return a configured PID Controller
void Robot::InitializePIDController(rev::CANPIDController pid_controller) {
    
    // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  pid_controller.SetP(P.get());
  pid_controller.SetI(I.get());
  pid_controller.SetD(D.get());
  pid_controller.SetOutputRange(-1, 1);
  /**
   * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
   * the pid controller in Smart Motion mode
   * - SetSmartMotionMinOutputVelocity() will put a lower bound in
   * RPM of the pid controller in Smart Motion mode
   * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
   * of the pid controller in Smart Motion mode
   * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
   * error for the pid controller in Smart Motion mode
   */
  pid_controller.SetSmartMotionMaxVelocity(kMaxVel);
  pid_controller.SetSmartMotionMinOutputVelocity(kMinVel);
  pid_controller.SetSmartMotionMaxAccel(kMaxAcc);
  pid_controller.SetSmartMotionAllowedClosedLoopError(kAllErr);
}
double Robot::ProcessControllerInput(double val) {
  return fabs(val) < Deadzone.get() ? 0 : val;
}
void Robot::RobotPeriodic() {
  double forward = -ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(LEFT));
  double targetVelocity = forward * MaxRPM.get();

  left_lead_controller.SetReference(targetVelocity, rev::ControlType::kSmartVelocity);
  right_lead_controller.SetReference(targetVelocity, rev::ControlType::kSmartVelocity);

  frc::SmartDashboard::PutNumber("Left Encoder Position", left_lead_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Left Motor Output", left_lead_motor.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Right Encoder Position", right_lead_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Motor Output", right_lead_motor.GetAppliedOutput());
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