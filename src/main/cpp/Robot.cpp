#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::rainbow(){
    // For every pixel

    std::cout << "rainbow" << std::endl;
    static int firstPixelHue = 0;
    auto pixelHue = (firstPixelHue + (0 * 180 / kLength)) % 180;
    for (int i = 0; i < kLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to process
      pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
      // Set the value
      a_leds[i].SetHSV(pixelHue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // Check bounds
    firstPixelHue %= 180;   
    led.SetData(a_leds);
}

void Robot::red(){
  std::cout << "red" << std::endl;
  for (int i = 0; i < kLength; i++){

    a_leds[i].SetHSV(0,100,100);
    
  }
  led.SetData(a_leds);
}

void Robot::RobotInit() {
    //LED Stuff
    led.SetLength(kLength);    
    led.SetData(a_leds);





    // clear the motor config
    LLeadMotor.RestoreFactoryDefaults();
    LFollowMotor.RestoreFactoryDefaults();
    RLeadMotor.RestoreFactoryDefaults();
    RFollowMotor.RestoreFactoryDefaults();
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/C%2B%2B/Smart%20Motion%20Example/src/main/cpp/Robot.cpp
    // basic initialization

    LFollowMotor.Follow(LLeadMotor);
    RFollowMotor.Follow(RLeadMotor);

    prefs.PutDouble("deadzone", 0.1);
    prefs.PutInt("maxrpm", 2400);
    prefs.PutDouble("p",1.0);
    prefs.PutDouble("i",0.0);
    prefs.PutDouble("d",0.0);
}
// return a configured PID Controller
void Robot::InitializePIDController(rev::CANPIDController pid_controller) {
    
  // default smart motion coefficients
  double kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  pid_controller.SetOutputRange(-1, 1);
  pid_controller.SetSmartMotionMinOutputVelocity(kMinVel);
  pid_controller.SetSmartMotionMaxAccel(kMaxAcc);
  pid_controller.SetSmartMotionAllowedClosedLoopError(kAllErr);
  pid_controller.SetSmartMotionMaxVelocity(prefs.GetInt("maxrpm"));
  // rock paper sissors pew pew shoot!
  pid_controller.SetP(prefs.GetInt("p"));
  pid_controller.SetI(prefs.GetInt("i"));
  pid_controller.SetD(prefs.GetInt("d"));
}
double Robot::ProcessControllerInput(double val) {
  return fabs(val) < prefs.GetDouble("deadzone") ? 0 : val;
}
void Robot::RobotPeriodic() {
  double speed = -ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(LEFT));
  double targetVelocity = speed * prefs.GetInt("maxrpm");
  frc::SmartDashboard::PutNumber("Speed", speed);
  frc::SmartDashboard::PutNumber("Turn", turn);


  red();

  drivetrain.ArcadeDrive(targetVelocity, turn, true);

  // don't read, just display
  frc::SmartDashboard::PutNumber("left lead encoder", LLead.GetEncoder());
  frc::SmartDashboard::PutNumber("right lead encoder", RLead.GetEncoder());
  frc::SmartDashboard::PutNumber("left motor applied", LLead.GetApplied());
  frc::SmartDashboard::PutNumber("right lead applied", RLead.GetApplied());
}


void Robot::AutonomousInit() {
  InitializePIDController(LLeadPID);
  InitializePIDController(RLeadPID);
}

void Robot::AutonomousPeriodic() {
 
}

void Robot::TeleopInit() {
  InitializePIDController(LLeadPID);
  InitializePIDController(RLeadPID);
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif