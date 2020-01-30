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
	prefs.PutInt("maxrpm", 2400);
	prefs.PutDouble("p",1.0);
	prefs.PutDouble("i",0.0);
	prefs.PutDouble("d",0.0);
	prefs.PutBoolean("use encoder", false);
	prefs.PutInt("flywheel error", 0);
	// Configure flywheel
	flywheelMotor.ConfigFactoryDefault();
	flywheelMotor.Config_kP(0, .05);
	flywheelMotor.Config_kF(0, .1);
	flywheelMotor.Config_kI(0, 6E-05);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(true);
	frc::SmartDashboard::PutNumber("Line Break Sensor", 2);
	prefs.PutInt("shooter output in ticks", 8000);
	
	// configure intake/shooter
	prefs.PutBoolean("wait to shoot", false);
	SmartDashboard::PutNumber("intake motor", 0);
	SmartDashboard::PutNumber("intake belt", 0);
	SmartDashboard::PutNumber("shooter belt", 0);
	// prefs.PutDouble("intake motor speed", 0.2);
	// prefs.PutDouble("intake belt speed", 0.2);
	// prefs.PutDouble("shooter belt speed", 0.2);
	// prefs.PutDouble("shooter belt speed reverse", 0.2);
	// output vision \ing values
	MakeSlider("lowerH", 15, 179);
	MakeSlider("lowerS", 100);
	MakeSlider("lowerV", 130);
	MakeSlider("upperH", 60, 179);
	MakeSlider("upperS", 255);
	MakeSlider("upperV", 255);
	prefs.PutDouble("color spinner motor speed",0.5);

	// initialize color motor
	colorMatcher.AddColorMatch(aimRed);
	colorMatcher.AddColorMatch(aimYellow);
	colorMatcher.AddColorMatch(aimBlue);
	colorMatcher.AddColorMatch(aimGreen);

	// Vision Camera
	Shuffleboard::GetTab("vision")
	.Add("Front Camera", true)
	.WithWidget(BuiltInWidgets::kToggleButton);
		
}

// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {
	double speed = ProcessControllerInput(pilot.GetY(LEFT));
	double turn = ProcessControllerInput(pilot.GetX(RIGHT));
	double targetVelocity = speed;
	drivetrain.ArcadeDrive(targetVelocity, -turn, true);


	// Output useful values
	frc::SmartDashboard::PutNumber("Current L motor velocity", LLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Current R motor velocity", RLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Desired", speed * 5500);
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
	frc::SmartDashboard::PutNumber("Line Break Sensor", 2);
}
	
void Robot::TeleopInit() {
  RLead.UseEncoder(prefs.GetBoolean("use encoder"));
  LLead.UseEncoder(prefs.GetBoolean("use encoder"));
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
	HandleStuff();

	//manage intake state, toggle with a button

}
void Robot::HandleStuff() {
	bool isIntaking = ProcessControllerInput(copilot.GetTriggerAxis(LEFT)) > 0;
	bool runShooterBelt = ProcessControllerInput(copilot.GetTriggerAxis(RIGHT)) > 0;
	int error = std::abs(flywheelMotor.GetClosedLoopError());

	// canIntake.toggle(copilot.GetXButton());
	// isShooting.toggle(copilot.GetAButton());

	// SmartDashboard::PutBoolean("isIntaking", isIntaking);
	// SmartDashboard::PutBoolean("canIntake", canIntake);
	// SmartDashboard::PutBoolean("wants to shoot", isShooting);

	// intakePiston.Set(canIntake);
	// intakeMotor.Set(ControlMode::PercentOutput, isIntaking ? prefs.GetDouble("intake belt speed",0) : 0);
	// shooterBelt.Set(ControlMode::PercentOutput, isIntaking ? prefs.GetDouble("shooter belt speed",0) : 0);
	intakeMotor.Set(ControlMode::PercentOutput, SmartDashboard::GetNumber("intake motor", 0));
	intakeBelt.Set(ControlMode::PercentOutput, SmartDashboard::GetNumber("intake belt", 0));
	shooterBelt.Set(ControlMode::PercentOutput, SmartDashboard::GetNumber("shooter belt", 0));
	// turn on shooter belt
	// if (runShooterBelt) {
	// 	shooterBelt.Set(ControlMode::PercentOutput, prefs.GetDouble("shooter belt speed", 0));
	// }

	// if (isShooting) {
	// 	if (error > prefs.GetInt("flywheel error", 0)) {
	// 		shooterBelt.Set(ControlMode::PercentOutput, isIntaking ? -prefs.GetDouble("shooter belt speed reverse",0) : 0);
	// 	}
	if (pilot.GetAButton()) {
		flywheelMotor.Set(TalonFXControlMode::Velocity, prefs.GetInt("shooter output in ticks", 0));
	} else {
		flywheelMotor.Set(TalonFXControlMode::Velocity, 0);
	}
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