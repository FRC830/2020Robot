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
    flywheelMotor.ConfigFactoryDefault();
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
    // colorWheelMotor.SetNeutralMode(NeutralMode::Brake);
    colorMatcher.AddColorMatch(aimRed);
    colorMatcher.AddColorMatch(aimYellow);
    colorMatcher.AddColorMatch(aimBlue);
    colorMatcher.AddColorMatch(aimGreen);
    Shuffleboard::GetTab("vision")
    .Add("Front Camera", true)
    .WithWidget(BuiltInWidgets::kToggleButton);
    SmartDashboard::PutNumber("Velocity in ticks", 0);
    
    flywheelMotor.Config_kP(0, .05);
    flywheelMotor.Config_kF(0, .1);
    flywheelMotor.ConfigClosedloopRamp(2);    
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
std::tuple<char, double> Robot::ClosestColor() {
  frc::Color detectedColor = colorSensor.GetColor();
  char color;
  double confidence;
  frc::Color matchedColor = colorMatcher.MatchClosestColor(detectedColor, confidence);
  if (matchedColor == aimBlue) {
    color = 'B';
  } else if (matchedColor == aimRed) {
    color = 'R';
  } else if (matchedColor == aimGreen) {
    color = 'G';
  } else if (matchedColor == aimYellow) {
    color = 'Y';
  } else {
    color = 'N';
  }
  return std::make_tuple(color, confidence);
}
// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {
  double speed = 0; //-ProcessControllerInput(pilot.GetY(LEFT));
  double turn = ProcessControllerInput(pilot.GetX(RIGHT));
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

  ledStrip.Set(ledMode % ledStrip.NumModes());
  frc::SmartDashboard::PutString("current LED mode", ledStrip.Get());
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
void Robot::HandleVision() {
  // double toggleMode = ProcessControllerInput(pilot.GetTriggerAxis(LEFT));
  // if (toggleMode > 0) {
  //   // SmartDashboard::PutNumber("")
  //   Shuffleboard::GetTab("vision")
  //   .add("Camera", CameraServer::GetInstance()::GetServer("Front Camera"))
  // }
  // if toggleMode 
}

void Robot::TeleopPeriodic() {
  // color
  HandleLEDStrip();
  HandleDrivetrain();
  HandleColorWheel();
  double rpm = SmartDashboard::GetNumber("Velocity in ticks", 0);
  flywheelMotor.Set(TalonFXControlMode::Velocity, rpm);
}
void Robot::HandleColorWheel() {
  std::string gameData;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  char closestColor;
  double confidence;
  std::tie(closestColor, confidence) = ClosestColor();
  int proximity = (int) colorSensor.GetProximity();
  SmartDashboard::PutNumber("Proximity", proximity);
  SmartDashboard::PutNumber("Confidence", confidence);
  SmartDashboard::PutString("Closest Color", std::string(1, closestColor));
  if(gameData.length() > 0) {
    currentColorTarget = gameData[0];
  }
  if ((currentColorTarget == closestColor) || closestColor == 'N' || currentColorTarget == 'N') {
    colorWheelMotor.Set(ControlMode::PercentOutput, 0);
  } else {
    colorWheelMotor.Set(ControlMode::PercentOutput, -prefs.GetDouble("color spinner motor speed"));
  }

}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif