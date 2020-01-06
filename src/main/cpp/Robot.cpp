#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
void Robot::RobotInit() {
    // clear the motor config
    left_lead_motor.RestoreFactoryDefaults();
    left_follow_motor.RestoreFactoryDefaults();
    right_lead_motor.RestoreFactoryDefaults();
    right_follow_motor.RestoreFactoryDefaults();
    left_lead_motor.Set();
    left_follow_motor.Set();
    right_lead_motor.Set();
    right_follow_motor.Set();

    // Follow motors
    left_follow_motor.Follow(left_lead_motor);
    right_follow_motor.Follow(right_lead_motor);
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp

    rev::CANPIDController left_lead_controller = CreatePIDController(left_follow_motor);
    left_lead_encoder = left_lead_motor.GetEncoder();
    right_lead_encoder = right_lead_motor.GetEncoder();
    
    rev::CANPIDController right_lead_controller = CreatePIDController(left_follow_motor);
}
rev::CANPIDController Robot::CreatePIDController(rev::CANSparkMax motor) {
    
    // return a configured PID Controller
    // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  rev::CANPIDController pid_controller = motor.GetPIDController();
  pid_controller.SetP(P.get());
  pid_controller.SetI(I.get());
  pid_controller.SetD(D.get());
  pid_controller.SetOutputRange(-1, 1);
    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
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
  return pid_controller;
}

void Robot::RobotPeriodic() {
  double forward = fabs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : -pilot.GetY(LEFT);
  double setpoint = forward * 50
  left_lead_controller.SetReference(setpoint, rev::ControlType::kVelocity);
  right_lead_controller.SetReference(setpoint, rev::ControlType::kVelocity);
  ProcessVariable = m_encoder.GetPosition();
  frc::SmartDashboard::PutNumber("Process Variable", ProcessVariable);
  frc::SmartDashboard::PutNumber("Output", m_motor.GetAppliedOutput());
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