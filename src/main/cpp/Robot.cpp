#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::rainbow(){
    // For every pixel

    frc::SmartDashboard::PutString("MODE", "Rainbow")
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
  frc::SmartDashboard::PutString("MODE","RED")
  frc::SmartDashboard::PutNumber("LED COUNT", kLength)
  for (int i = 0; i < kLength; i++){

    a_leds[i].SetHSV(0,100,100);
    
  }
  led.SetData(a_leds);
}

void Robot::RobotInit() {
    //LED Stuff
    led.SetLength(kLength);    
    led.SetData(a_leds);
    led.Start();
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
    // output vision testing values
    MakeSlider("lowerH", 15, 179);
    MakeSlider("lowerS", 100);
    MakeSlider("lowerV", 130);
    MakeSlider("upperH", 60, 179);
    MakeSlider("upperS", 255);
    MakeSlider("upperV", 255);
    // initialize color motor
    srxMotor.SetNeutralMode(NeutralMode::Brake);
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
std::string Robot::ClosestColor() {

  
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
    colorString = "Unknown";
  }
  return colorString;
}
void Robot::RobotPeriodic() {
  // Drivetrain
  double speed = -ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(LEFT));
  double targetVelocity = speed * prefs.GetInt("maxrpm");
  frc::SmartDashboard::PutNumber("Speed", speed);
  frc::SmartDashboard::PutNumber("Turn", turn);


  red();

  drivetrain.ArcadeDrive(targetVelocity, turn, true);
  frc::SmartDashboard::PutNumber("left lead encoder", LLead.GetEncoder());
  frc::SmartDashboard::PutNumber("right lead encoder", RLead.GetEncoder());
  frc::SmartDashboard::PutNumber("left motor applied", LLead.GetApplied());
  frc::SmartDashboard::PutNumber("right lead applied", RLead.GetApplied());
  //min radius for vision
  frc::SmartDashboard::PutNumber("Min Radius", 1);
  frc::SmartDashboard::PutNumber("Original Radius", 7);

  // Color Wheel
  int proximity = (int) colorSensor.GetProximity();
  SmartDashboard::PutNumber("Proximity", proximity);
  SmartDashboard::PutString("Closest Color", ClosestColor());
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
  double rt = pilot.GetTriggerAxis(::GenericHID::kRightHand);
  SmartDashboard::PutNumber("RT", rt);
  
  bool red = pilot.GetBButton();
  bool blue = pilot.GetXButton();
  bool green = pilot.GetAButton();
  bool yellow = pilot.GetYButton();
  
  std::string color = ClosestColor();

  if ((red && color != "red") || (yellow && color != "yellow") || (blue && color != "blue") || (green && color != "green"))
    srxMotor.Set(ControlMode::PercentOutput, -0.5);
  else  
    srxMotor.Set(ControlMode::PercentOutput, 0);
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif