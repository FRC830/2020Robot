#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "utility.h"
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
	ConfigurePIDF(flywheelMotor, .05, 6E-05, 0, .1);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(false);
	prefs.PutInt("flywheel speed", 8000);

	// configure intake
	ConfigurePIDF(intakeBelt, .03, 6E-05, 0, 0);
	intakeBelt.SetInverted(true);
	prefs.PutInt("intake belt", 0);
	prefs.PutDouble("intake roller speed",0.5);

	// Configure Line break sensors & belts
	frc::SmartDashboard::PutNumber("Line Break Sensor", 2);
	frc::SmartDashboard::PutNumber("Line Break Sensor 2", 2);
	prefs.PutDouble("shooter speed",0.0);
	prefs.PutDouble("reverse belt speed",0.0);
	shooterBelt.SetInverted(true);
	
	// Configure Vision
	frc::ShuffleboardTab &visionTab = Shuffleboard::GetTab("vision");
	MakeSlider(visionTab, "ballLowerH", 15, 179);
	MakeSlider(visionTab, "ballLowerS", 100);
	MakeSlider(visionTab, "ballLowerV", 130);
	MakeSlider(visionTab, "ballUpperH", 60, 179);
	MakeSlider(visionTab, "ballUpperS", 255);
	MakeSlider(visionTab, "ballUpperV", 255);
	MakeSlider(visionTab, "tapeLowerH", 15, 179);
	MakeSlider(visionTab, "tapeLowerS", 100);
	MakeSlider(visionTab, "tapeLowerV", 130);
	MakeSlider(visionTab, "tapeUpperH", 60, 179);
	MakeSlider(visionTab, "tapeUpperS", 255);
	MakeSlider(visionTab, "tapeUpperV", 255);

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

	double speed = ApplyDeadzone(pilot.GetY(LEFT), prefs.GetDouble("deadzone"));
	double turn = ApplyDeadzone(pilot.GetX(RIGHT), prefs.GetDouble("deadzone"));
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
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	ConfigurePIDF(LLeadPID, 0,0,0,0.0001755);
	ConfigurePIDF(RLeadPID, 0,0,0,0.0001755);

}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

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
	HandleIntake();

}

void Robot::HandleColorWheel() {
	std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	char closestColor;
	double confidence;
	std::tie(closestColor, confidence) = ClosestColor(colorSensor);
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
	double reverseSpeed = prefs.GetDouble("reverse belt speed",0);
	bool lineBreak1Broken = !lineBreak1.Get();
	bool lineBreak2Broken = !lineBreak2.Get();

	bool lineBreak3Broken = lineBreak3.Get();


	// The 'run in reverse because something bad happened'
	if (pilot.GetStartButton()) {
		shooterBelt.Set(ControlMode::PercentOutput, -reverseSpeed);
		intakeBelt.Set(ControlMode::PercentOutput, -reverseSpeed);
		return; // do not run any other shooter code
	}
	// The 'run in reverse to move ball back to the front'
	if (pilot.GetBackButton()){
		if (!lineBreak2Broken){
			shooterBelt.Set(ControlMode::PercentOutput, -reverseSpeed);
			intakeBelt.Set(ControlMode::PercentOutput, -reverseSpeed);
		} else {
			shooterBelt.Set(ControlMode::PercentOutput, 0);
			intakeBelt.Set(ControlMode::PercentOutput, 0);
		}
		// already handled sensor here, prevent it from being handled later
		lineBreak2WasBroken = lineBreak2Broken;
		return;
	}

	// read values
	double flywheelSpeedVelocity = prefs.GetInt("flywheel speed", 0);
	double shooterSpeedPercent = prefs.GetDouble("shooter speed", 0);
	double intakeSpeedVelocity = prefs.GetInt("intake speed", 0);

	// The 'spin flywheel' functionality
	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0), flywheelSpeedVelocity);
	bool meetsThreshold = (error <= .1);
	
	bool runFlywheel = ApplyDeadzone(pilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	if (runFlywheel) {
		flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelSpeedVelocity);
		if (meetsThreshold) {
			isUpToSpeed = true;
		}
	} else {
		if (!meetsThreshold) {
			isUpToSpeed = false;
		}
		flywheelMotor.Set(TalonFXControlMode::PercentOutput, 0);
	}
	
	// Logging time!!!
	SmartDashboard::PutNumber("Line Break Sensor 1", lineBreak1.Get());
	SmartDashboard::PutNumber("Line Break Sensor 2", lineBreak2.Get());
	SmartDashboard::PutNumber("Line Break Sensor 3", lineBreak3.Get());
	SmartDashboard::PutNumber("current intake velocity", intakeBelt.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("current flywheel velocity", flywheelMotor.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("Balls Stored", ballsStored);
	SmartDashboard::PutNumber("Balls Shot", ballsShot);

	// The 'shoot' functionality
	SmartDashboard::PutNumber("current error", std::fabs(flywheelMotor.GetClosedLoopError(0)));
	if (pilot.GetAButton()) {
		if (meetsThreshold) {
			isUpToSpeed = true;
		}
		flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelSpeedVelocity);
		if (isUpToSpeed) {
			shooterBelt.Set(ControlMode::PercentOutput, shooterSpeedPercent);
			intakeBelt.Set(ControlMode::Velocity, intakeSpeedVelocity);
		}
		return; // do not run intake code
	}

	// The 'intake' functionality
	// If the line break sensor detects the retroreflective tape, It will have a value of 1.0000
	// If the line break sensor detects something infront of the retroreflective tape, it will have a value of 0.0000
	if (!lineBreak1Broken && !lineBreak2Broken) {
		intakeBelt.Set(ControlMode::PercentOutput, 0);
		shooterBelt.Set(ControlMode::PercentOutput, 0);
	} else {
		intakeBelt.Set(ControlMode::Velocity, intakeSpeedVelocity);
		shooterBelt.Set(ControlMode::PercentOutput, shooterSpeedPercent);
	}

	frc::SmartDashboard::PutBoolean("meets threshold", meetsThreshold);
	frc::SmartDashboard::PutBoolean("is up to speed", isUpToSpeed);


	//Deals with ball count
	//Pseudocode: ✔️Increase ball count when line break sensor 2 is broken.
	//			  ✔️Decrease ball count when line break sensor 3 is broken.
	//			  When there is space between the back ball and the second line break sensor, run belts backwards until it intercepts it
	//			  *This will increase the ball count by 1, in which case we decrease the ball count by one to correct it
	//		   	  *Run the belts forward to then move the ball out of the second line break sensor


	if (lineBreak2Broken && !lineBreak2WasBroken){

		ballsStored += 1;

	}
	if (lineBreak3Broken && !lineBreak3WasBroken){

		ballsStored -= 1;
		ballsShot += 1;

	}

	lineBreak1WasBroken = lineBreak1Broken;
	lineBreak2WasBroken = lineBreak2Broken;
	lineBreak3WasBroken = lineBreak3Broken;

}

void Robot::HandleIntake(){
	//Change to copilot later
	bool isIntaking = ApplyDeadzone(pilot.GetTriggerAxis(LEFT), 0.2) > 0;
	intakePiston.Set(isIntaking);
	double intakeRollerSpeed = prefs.GetDouble("intake roller speed", 0.5);
	if (isIntaking){
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
	} else {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif