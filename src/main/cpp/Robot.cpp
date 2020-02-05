#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
void Robot::RobotInit() {
	// Configure drivetrain
	LLeadMotor.RestoreFactoryDefaults();
	LFollowMotor.RestoreFactoryDefaults();
	RLeadMotor.RestoreFactoryDefaults();
	RFollowMotor.RestoreFactoryDefaults();
	LFollowMotor.Follow(LLeadMotor, false);
	RFollowMotor.Follow(RLeadMotor, false); // differential drive inverts the right motor
	prefs.PutDouble("deadzone", 0.1);
	prefs.PutBoolean("use encoder", false);
	// Configure flywheel
	flywheelMotor.ConfigFactoryDefault();
	flywheelMotor.Config_kP(0, .05);
	flywheelMotor.Config_kF(0, .1);
	flywheelMotor.Config_kI(0, 6E-05);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(false);
	prefs.PutInt("flywheel speed", 8000);

	// Configure Line break sensors & belts
	frc::SmartDashboard::PutNumber("Line Break Sensor", 2);
	frc::SmartDashboard::PutNumber("Line Break Sensor 2", 2);
	frc::SmartDashboard::PutNumber("intake belt",0);
	frc::SmartDashboard::PutNumber("shooter belt",0);
	shooterBelt.SetInverted(true);
	intakeBelt.SetInverted(true);
	
	// Configure Vision
	MakeSlider("ballLowerH", 15, 179);
	MakeSlider("ballLowerS", 100);
	MakeSlider("ballLowerV", 130);
	MakeSlider("ballUpperH", 60, 179);
	MakeSlider("ballUpperS", 255);
	MakeSlider("ballUpperV", 255);
	
	MakeSlider("tapeLowerH", 15, 179);
	MakeSlider("tapeLowerS", 100);
	MakeSlider("tapeLowerV", 130);
	MakeSlider("tapeUpperH", 60, 179);
	MakeSlider("tapeUpperS", 255);
	MakeSlider("tapeUpperV", 255);

	Shuffleboard::GetTab("vision")
	.Add("Front Camera", true)
	.WithWidget(BuiltInWidgets::kToggleButton);
	// Configure Color Sensor
	prefs.PutDouble("color spinner motor speed",0.5);
	colorMatcher.AddColorMatch(aimRed);
	colorMatcher.AddColorMatch(aimYellow);
	colorMatcher.AddColorMatch(aimBlue);
	colorMatcher.AddColorMatch(aimGreen);		
}

// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {

	double speed = ProcessControllerInput(pilot.GetY(LEFT));
	double turn = ProcessControllerInput(pilot.GetX(RIGHT));
	drivetrain.ArcadeDrive(speed, -turn, true);

	// Output useful values
	frc::SmartDashboard::PutNumber("Current L motor velocity", LLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Current R motor velocity", RLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Desired Speed (Velocity)", speed * 5500);
	frc::SmartDashboard::PutNumber("Turn", turn);
	frc::SmartDashboard::PutNumber("left lead position", LLead.GetPosition());
	frc::SmartDashboard::PutNumber("right lead position", RLead.GetPosition());
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
  RLead.UseEncoder(prefs.GetBoolean("use encoder"));
  LLead.UseEncoder(prefs.GetBoolean("use encoder"));
	InitializePIDController(LLeadPID);
	InitializePIDController(RLeadPID);
        frc::SmartDashboard::PutNumber("Intake Belt Min Velocity", 100);
        frc::SmartDashboard::PutNumber("Intake Belt Max Velocity", 999999);
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
	// HandleColorWheel(); // Currently breaks robot code w/o sensor
	HandleShooter();
	//manage intake state, toggle with a button

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
// Handle Line Sensor Indexing
void Robot::HandleShooter() {
	bool runFlywheel = ProcessControllerInput(pilot.GetTriggerAxis(RIGHT)) > 0;
	if (runFlywheel) {
		flywheelMotor.Set(TalonFXControlMode::Velocity, prefs.GetInt("flywheel speed", 0));
	} else {
		flywheelMotor.Set(TalonFXControlMode::PercentOutput, 0);
	}
	// Returns 1 if NOT blocked, so runs when neither are blocked
	frc::SmartDashboard::PutNumber("Line Break Sensor", lineBreak.Get());
	frc::SmartDashboard::PutNumber("Line Break Sensor 2", lineBreak2.Get());
	intakeBeltSpeed = SmartDashboard::GetNumber("intake belt", 0);
	shooterBeltSpeed = SmartDashboard::GetNumber("shooter belt", 0);
	double maxVelocity = SmartDashboard::GetNumber("Intake Belt Min Velocity",100);
	double minVelocity = SmartDashboard::GetNumber("Intake Belt Max Velocity",99999); // impossibly high
	if (lineBreak.Get() && lineBreak2.Get()) {
		intakeBeltSpeed = 0.0;
		shooterBeltSpeed = 0.0;
	} else {
		double currentVelocity = intakeBelt.GetSelectedSensorVelocity(0);
		SmartDashboard::PutNumber("current velocity",currentVelocity);
		if (currentVelocity < minVelocity) {
			intakeBeltSpeed+=.1;
		} else if (currentVelocity > maxVelocity) {
			intakeBeltSpeed-=.1;
		}
	}

	intakeBelt.Set(ControlMode::PercentOutput, intakeBeltSpeed);
	shooterBelt.Set(ControlMode::PercentOutput, shooterBeltSpeed);
}
/*
 _    _ _______ _____ _      _____ _________     __  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
| |  | |__   __|_   _| |    |_   _|__   __\ \   / / |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
| |  | |  | |    | | | |      | |    | |   \ \_/ /  | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
| |  | |  | |    | | | |      | |    | |    \  /    |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
| |__| |  | |   _| |_| |____ _| |_   | |     | |    | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) 
|\____/   |_|  |_____|______|_____|  |_|     |_|    |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/                                            
*/
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
	pid_controller.SetOutputRange(-1, 1);
	pid_controller.SetP(0);
	pid_controller.SetI(0);
	pid_controller.SetD(0);
	pid_controller.SetFF(0.0001755);
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
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif