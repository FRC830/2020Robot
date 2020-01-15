#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::RobotInit() {
    // clear the motor config
    LLeadMotor.RestoreFactoryDefaults();
    LFollowMotor.RestoreFactoryDefaults();
    RLeadMotor.RestoreFactoryDefaults();
    RFollowMotor.RestoreFactoryDefaults();
    // have rear motors follow front motors

    LFollowMotor.Follow(LLeadMotor);
    RFollowMotor.Follow(RLeadMotor);
    // output preference values    
    prefs.PutDouble("deadzone", 0.1);
    prefs.PutInt("maxrpm", 2400);
    prefs.PutDouble("p",1.0);
    prefs.PutDouble("i",0.0);
    prefs.PutDouble("d",0.0);
    prefs.PutDouble("color spinner motor speed",0.5);
    // output vision testing values
    MakeSlider("lowerH", 15, 179);
    MakeSlider("lowerS", 100);
    MakeSlider("lowerV", 130);
    MakeSlider("upperH", 60, 179);
    MakeSlider("upperS", 255);
    MakeSlider("upperV", 255);
    frc::SmartDashboard::PutNumber("Original", 1.0);
    // initialize color motor
    colorWheelMotor.SetNeutralMode(NeutralMode::Brake);
    colorMatcher.AddColorMatch(aimRed);
    colorMatcher.AddColorMatch(aimYellow);
    colorMatcher.AddColorMatch(aimBlue);
    colorMatcher.AddColorMatch(aimGreen);
    
}
// adds a configured slider to vision tab
void Robot::MakeSlider(std::string name, double defaultV, double max) {
    wpi::StringMap<std::shared_ptr<nt::Value>> properties {
        std::make_pair("min", nt::Value::MakeDouble(0)),
        std::make_pair("max", nt::Value::MakeDouble(max))
    };
    Shuffleboard::GetTab("vision")
    .Add(name, defaultV)
    .WithWidget(BuiltInWidgets::kNumberSlider)
    .WithProperties(properties);
}
// configures a PID controller
void Robot::InitializePIDController(rev::CANPIDController pid_controller) {
    
  // default smart motion coefficients
  double kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  pid_controller.SetOutputRange(-1, 1);
  pid_controller.SetSmartMotionMinOutputVelocity(kMinVel);
  pid_controller.SetSmartMotionMaxAccel(kMaxAcc);
  pid_controller.SetSmartMotionAllowedClosedLoopError(kAllErr);
  pid_controller.SetSmartMotionMaxVelocity(prefs.GetInt("maxrpm"));
  pid_controller.SetP(prefs.GetInt("p"));
  pid_controller.SetI(prefs.GetInt("i"));
  pid_controller.SetD(prefs.GetInt("d"));
}
// apply deadzone & possible scaling, etc
double Robot::ProcessControllerInput(double val) {
  return fabs(val) < prefs.GetDouble("deadzone") ? 0 : val;
}

// Return the closest detected color
std::tuple<std::string, double> Robot::ClosestColor() {
  frc::Color detectedColor = colorSensor.GetColor();
  std::string colorString;
  double confidence;
  frc::Color matchedColor = colorMatcher.MatchClosestColor(detectedColor, confidence);
  if (matchedColor == aimBlue) {
    colorString = "blue";
  } else if (matchedColor == aimRed) {
    colorString = "red";
  } else if (matchedColor == aimGreen) {
    colorString = "green";
  } else if (matchedColor == aimYellow) {
    colorString = "yellow";
  } else {
    colorString = "unknown";
  }
  return std::make_tuple(colorString, confidence);
}
// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {
  double speed = -ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(LEFT));
  double targetVelocity = speed * prefs.GetInt("maxrpm");
  frc::SmartDashboard::PutNumber("Speed", speed);
  frc::SmartDashboard::PutNumber("Turn", turn);
  drivetrain.ArcadeDrive(targetVelocity, turn, true);
  frc::SmartDashboard::PutNumber("left lead encoder", LLead.GetEncoder());
  frc::SmartDashboard::PutNumber("right lead encoder", RLead.GetEncoder());
  frc::SmartDashboard::PutNumber("left motor applied", LLead.GetApplied());
  frc::SmartDashboard::PutNumber("right lead applied", RLead.GetApplied());
}
// Handle LED Strip code
void Robot::HandleLEDStrip() {
  double angle = pilot.GetPOV();
  if (angle >= 45 && angle <= 135) {
    ledMode--;
  } else if (angle >= 225 && angle <= 315 ) {
    ledMode++;
  } else {
    return;
  }

  ledStrip.Set(ledMode % (int) LEDModes::COUNT);
}
void Robot::RobotPeriodic() {



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


void Robot::TeleopPeriodic() {
  // color
  HandleLEDStrip();
  HandleDrivetrain();
  HandleColorWheel();
  // use this
  double rpm = 0;
  if (pilot.GetAButton()) {
    rpm = ProcessControllerInput(pilot.GetY(RIGHT)) * 500.0;
  } else {
    rpm = SmartDashboard::GetNumber("Velocity in RPM", 0);
  } // can alan check the led thing
  // double speed = ProcessControllerInput(pilot.GetY(RIGHT));
  falcon.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, rpm);
}
void Robot::HandleColorWheel() {
  int proximity = (int) colorSensor.GetProximity();
  SmartDashboard::PutNumber("Proximity", proximity);
  bool red = pilot.GetBButton();
  bool blue = pilot.GetXButton();
  bool green = pilot.GetAButton();
  bool yellow = pilot.GetYButton();
  std::string color;
  double confidence;
  // https://www.techiedelight.com/return-multiple-values-functions-cpp/
  std::tie(color, confidence) = ClosestColor();
  SmartDashboard::PutNumber("Confidence", confidence);
  SmartDashboard::PutString("Closest Color", color);
  if ((red && color != "red") || (yellow && color != "yellow") || (blue && color != "blue") || (green && color != "green")) {
    colorWheelMotor.Set(ControlMode::PercentOutput, -prefs.GetDouble("color spinner motor speed"));
  } else {
    colorWheelMotor.Set(ControlMode::PercentOutput, 0);
  }  
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif