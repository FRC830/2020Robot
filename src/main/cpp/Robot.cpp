#include "Robot.h"


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
	// Configure flywheel
	ConfigurePIDF(flywheelMotor, .05, 6E-05, 0, .1);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(false);

	frc::ShuffleboardTab &visionTab = frc::Shuffleboard::GetTab("vision");
	
	// configure intake
	ConfigurePIDF(intakeBelt, .03, 6E-05, 0, 0);
	intakeBelt.SetInverted(true);

	// Configure Line break sensors & belts
	frc::SmartDashboard::PutNumber("Line Break Sensor", 2);
	frc::SmartDashboard::PutNumber("Line Break Sensor 2", 2);
	shooterBelt.SetInverted(true);
	
	// Configure Vision
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

	visionTab2->PutBoolean("Front Camera", true);
	// Configure Color Sensor
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
	double angle = copilot.GetPOV();
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

void Robot::HandleCamera() {
	if (pilot.GetBumperPressed(LEFT)) {
		frontCamera = true;
	} else if (pilot.GetBumperPressed(RIGHT)) { 
		frontCamera = false;
	}
	visionTab2->PutBoolean("Front Camera", frontCamera);
}

void Robot::TeleopPeriodic() {
	HandleLEDStrip();
	HandleDrivetrain();
	// HandleColorWheel(); // Currently breaks robot code w/o sensor
	HandleShooter();
	HandleIntake();
	HandleCamera();
	HandleElevator();

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
		colorWheelMotor.Set(ControlMode::PercentOutput, -colorSpinnerSpeed);
	}

}


// Handle Line Sensor Indexing
void Robot::HandleShooter() {
	bool lineBreak1Broken = lineBreak1.Get();
	bool lineBreak2Broken = lineBreak2.Get();
	bool lineBreak3Broken = lineBreak3.Get();
	


	// Start: The 'run in reverse because something bad happened'
	if (copilot.GetStartButton()) {
		shooterBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
		return; // do not run any other shooter code
	}
	// Back: The 'run in reverse to move ball back to the front'
	if (copilot.GetBackButton()){
		if (!lineBreak2Broken){
			shooterBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
			intakeBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
			intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);

		} else {
			shooterBelt.Set(ControlMode::PercentOutput, 0);
			intakeBelt.Set(ControlMode::PercentOutput, 0);
		}
		// already handled sensor here, prevent it from being handled later
		lineBreak2WasBroken = lineBreak2Broken;
		return;
	}
	// The 'spin flywheel' functionality
	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0), flywheelSpeedVelocity);
	bool meetsThreshold = (error <= .1);
	
	bool runFlywheel = ApplyDeadzone(copilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
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
	
	// Log variables
	SmartDashboard::PutBoolean("Line Break Sensor 1", lineBreak1Broken);
	SmartDashboard::PutBoolean("Line Break Sensor 2", lineBreak2Broken);
	SmartDashboard::PutBoolean("Line Break Sensor 3", lineBreak3Broken);
	SmartDashboard::PutNumber("current intake velocity", intakeBelt.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("current flywheel velocity", flywheelMotor.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("Balls Stored", ballsStored);
	SmartDashboard::PutNumber("Balls Shot", ballsShot);

	// The 'shoot' functionality
	SmartDashboard::PutNumber("current error", std::fabs(flywheelMotor.GetClosedLoopError(0)));
	if (copilot.GetAButton()) {
		if (meetsThreshold) {
			isUpToSpeed = true;
		}
		flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelSpeedVelocity);
		if (isUpToSpeed) {
			shooterBelt.Set(ControlMode::PercentOutput, shooterBeltSpeed);
			intakeBelt.Set(ControlMode::Velocity, intakeBeltSpeedVelocity);
		}
		return; // do not run intake code
	}

	// The 'intake' functionality
	if (!lineBreak1Broken && !lineBreak2Broken) {
		intakeBelt.Set(ControlMode::PercentOutput, 0);
		shooterBelt.Set(ControlMode::PercentOutput, 0);
	} else {
		intakeBelt.Set(ControlMode::Velocity, intakeBeltSpeedVelocity);
		shooterBelt.Set(ControlMode::PercentOutput, shooterBeltSpeed);
	}

	frc::SmartDashboard::PutBoolean("meets threshold", meetsThreshold);
	frc::SmartDashboard::PutBoolean("is up to speed", isUpToSpeed);

	if (lineBreak2Broken && !lineBreak2WasBroken){
		ballsStored++;
	}
	if (lineBreak3Broken && !lineBreak3WasBroken){
		ballsStored--;
		ballsShot++;
	}

	lineBreak1WasBroken = lineBreak1Broken;
	lineBreak2WasBroken = lineBreak2Broken;
	lineBreak3WasBroken = lineBreak3Broken;

}
void Robot::HandleElevator() {
	
}
void Robot::HandleIntake(){
	//Change to copilot later
	bool isIntaking = ApplyDeadzone(copilot.GetTriggerAxis(LEFT), 0.2) > 0;
	intakePiston.Set(isIntaking);
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