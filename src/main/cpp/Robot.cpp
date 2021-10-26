#include "Robot.h"																														

using namespace frc;
/*========================
 ___  ====================
|   | ====================
| | | INIT FUNCTIONS =====
|___| ====================
==========================
==========================*/
void Robot::RobotInit() {
	/*=============
	Drivetrain
	=============*/
	prefs.PutDouble("deadzone", 0.1);
	

	/*=============
	Flywheel
	=============*/
	ConfigurePIDF(flywheelMotor, .04, 6E-05, 0, 0);
	flywheelMotorFollow.Follow(flywheelMotor);
	flywheelMotorFollow.SetInverted(false);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(true);
	flywheelMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
	SmartDashboard::PutNumber("FLYWHEEL RPM",flywheelRPM);
	flywheelMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{true,20,30,0.5}); // if above 30A for .5s, limit to 20A
	/*=============
	Elevator
	=============*/
	elevatorMotor.ConfigFactoryDefault();
	elevatorMotor.SetInverted(true);
	elevatorMotor.SetSelectedSensorPosition(0);
	elevatorMotor.SetNeutralMode(motorcontrol::NeutralMode::Brake);
	/*=============
	Intake Motor + Belt
	=============*/
	ConfigurePIDF(belt, .03, 6E-05, 0, 0);
	belt.SetInverted(false);
	belt.SetNeutralMode(motorcontrol::NeutralMode::Brake);
	intakeMotor.SetInverted(true);
	SmartDashboard::PutNumber("INTAKE BELT SHOOT",beltFireTicks);

	/*=============
	LED Strip
	=============*/
	debugTab->PutNumber("LED MODE", ledMode);
	SmartDashboard::PutString("current LED mode", ledStrip.Get());

	/*=============
	Line break sensors
	=============*/
	debugTab->PutBoolean("Line Break Sensor 1", false);
	debugTab->PutBoolean("Line Break Sensor 2", false);
	debugTab->PutBoolean("Line Break Sensor 3", false);

	/*=============
	Vision
	=============*/
	frc::ShuffleboardTab &tempVisionTab = frc::Shuffleboard::GetTab("vision");
	MakeSlider(tempVisionTab, "ballLowerH", 15, 179);
	MakeSlider(tempVisionTab, "ballLowerS", 100);
	MakeSlider(tempVisionTab, "ballLowerV", 130);
	MakeSlider(tempVisionTab, "ballUpperH", 60, 179);
	MakeSlider(tempVisionTab, "ballUpperS", 255);
	MakeSlider(tempVisionTab, "ballUpperV", 255);
	MakeSlider(tempVisionTab, "tapeLowerH", 15, 179);
	MakeSlider(tempVisionTab, "tapeLowerS", 100);
	MakeSlider(tempVisionTab, "tapeLowerV", 130);
	MakeSlider(tempVisionTab, "tapeUpperH", 60, 179);
	MakeSlider(tempVisionTab, "tapeUpperS", 255);
	MakeSlider(tempVisionTab, "tapeUpperV", 255);
	visionTab->PutBoolean("Front Camera", true);

	/*=============
	Color Sensor
	=============*/
	colorMatcher.AddColorMatch(aimRed);
	colorMatcher.AddColorMatch(aimYellow);
	colorMatcher.AddColorMatch(aimBlue);
	colorMatcher.AddColorMatch(aimGreen);

	/*=============
	Auton
	=============*/
	autonChooser.SetDefaultOption(simpleAuton, simpleAuton);
	autonChooser.AddOption(middlePathAuton, middlePathAuton); // starts from center of field
	autonChooser.AddOption(straightPathAuton, straightPathAuton); // starts from right side of field, but driver sees it as left
	autonChooser.AddOption(noneAuton, noneAuton); // do nothing
	autonChooser.AddOption(basicAuton, basicAuton); // go forward, shoot, go back
	autonChooser.AddOption(simpleAuton, simpleAuton); // very simple, just get off line
	autonChooser.AddOption(specialAuton, specialAuton); // very simple, just get off line

  	frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

}

void Robot::AutonomousInit() {
	// TODO fix drivetrain

	SimpleTimeFromStart.Start();
	SimpleTimeFromStart.Reset();
	BasicTimer.Reset();
	gyro.Reset();
	stage = 0;
	timerStarted = false;
	std::string currentAutonMode = autonChooser.GetSelected();
	//odometry.ResetPosition(trajectory.InitialPose, gyro.GetAngle());
}

void Robot::DisabledInit() {
	//drivetrain.SetSafetyEnabled(true);

	// Make sure it is back in coast mode
	// TODO
}

void Robot::TeleopInit() {
	// reset elevator
	inputScale = 1.0;
	elevatorMotor.SetSelectedSensorPosition(0);
}
/*=========================
 ___   ====================
|_  |  ====================
 _| |_ PERIODIC FUNCTIONS =
|_____|====================
===========================
===========================*/
void Robot::RobotPeriodic() {}
void Robot::TestPeriodic() {}
void Robot::TeleopPeriodic() {
	HandleLEDStrip();
	HandleDrivetrain();
	HandleColorWheel(); 
	HandleIntake();
	HandleShooter();
	HandleVision();
	HandleElevator();
}
void Robot::AutonomousPeriodic() {
	std::string currentAutonMode = autonChooser.GetSelected();
	if (currentAutonMode == noneAuton) {
		// do nothing
	} else if (currentAutonMode == basicAuton) {
		// TODO
	}
}
/*==========================
 ___   drivetrain, belts ===
|_  | ======================
|  _| MAIN FUNCTIONALITY ===
|___| ======================
====== intake, elevator ====
============================*/

void Robot::HandleDrivetrain() {

	double fwd = ApplyDeadzone(pilot.GetY(LEFT), prefs.GetDouble("deadzone"))/3;
	double strafe = ApplyDeadzone(pilot.GetX(LEFT), prefs.GetDouble("deadzone"))/3;
	double turn = ApplyDeadzone(pilot.GetX(RIGHT), prefs.GetDouble("deadzone"))/3;
	SmartDashboard::PutNumber("fwd", fwd);
	SmartDashboard::PutNumber("strafe", strafe);
	SmartDashboard::PutNumber("turn", turn);
	swerve_drive.Feed(fwd, strafe, turn);
	swerve_drive.ApplyToSwerveModules();
}


// Handle Line Sensor Indexing
void Robot::HandleShooter() {
	
	debugTab->PutNumber("get current",flywheelMotor.GetOutputCurrent()); // above 30 is bad
	bool lineBreak1Broken = !lineBreak1.Get();
	bool lineBreak2Broken = !lineBreak2.Get();
	bool lineBreak3Broken = !lineBreak3.Get();

	debugTab->PutBoolean("Sensor 1 Broken", lineBreak1Broken);
	debugTab->PutBoolean("Sensor 2 Broken", lineBreak2Broken);
	debugTab->PutBoolean("Sensor 3 Broken", lineBreak3Broken);

	// Manual Belt controls
	SmartDashboard::PutNumber("read", copilot.GetY(LEFT));
	double manualBeltPower = ApplyDeadzone(-copilot.GetY(LEFT), .35);
	if (manualBeltPower != 0) {
		belt.Set(ControlMode::PercentOutput, manualBeltPower);
		double intakeSpeed = (manualBeltPower > 0) ? intakeRollerSpeed : -intakeRollerSpeed * 0.5;
		intakeMotor.Set(ControlMode::PercentOutput, intakeSpeed);
		return;
	}
	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor, flywheelRPM);
	bool meetsThreshold = (error <= .1);
	bool runFlywheel = ApplyDeadzone(copilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool runShooter = copilot.GetAButton();
	flywheelRPM = SmartDashboard::GetNumber("FLYWHEEL RPM",flywheelRPM);
	beltFireTicks = SmartDashboard::GetNumber("INTAKE BELT",beltFireTicks);
	// Log variables

	debugTab->PutNumber("current intake velocity", belt.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("current flywheel velocity", flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor);
	debugTab->PutNumber("current error", std::fabs(flywheelMotor.GetClosedLoopError(0)));
	// Update the status of up to speed
	if ((runShooter || runFlywheel) && meetsThreshold) {
		isUpToSpeed = true;
	}
	// this can be changed to an 'else if' to speed up shooting and decrease accuracy
	if (!meetsThreshold) {
		isUpToSpeed = false;
	}
	debugTab->PutBoolean("meets threshold", meetsThreshold);
	debugTab->PutBoolean("is up to speed", isUpToSpeed);
	// The 'spin flywheel && shoot' functionality
	ignoreCountingOut = true;
	if (runShooter && isUpToSpeed) { // fire!
		ignoreCountingOut = false;
		flywheelMotor.Set(TalonFXControlMode::Velocity, (int) (flywheelRPM / kTalonRPMConversionFactor));
		belt.Set(ControlMode::Velocity, beltFireTicks);
		//intakeMotor.Set(ControlMode::PercentOutput, 0.3);
		return;
	} else if (runFlywheel || (runShooter && !isUpToSpeed)) {
		if (lineBreak3Broken) { // run belts & flywheel back when touching sensor
			belt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
			flywheelMotor.Set(TalonFXControlMode::Velocity, (int) (-flywheelReverseRPM / kTalonRPMConversionFactor));
		} else { // otherwise run flywheel and stop running belt back
			flywheelMotor.Set(TalonFXControlMode::Velocity, (int) (flywheelRPM / kTalonRPMConversionFactor));
			belt.Set(ControlMode::PercentOutput, 0);
		}
		return;
	} else { // not spinning or shooting, so stop flywheel
		flywheelMotor.Set(TalonFXControlMode::Velocity, 0);
	}

	// The 'intake' functionality
	if (!lineBreak2Broken) { 
		ignoreCountingIn = false;
		belt.Set(ControlMode::PercentOutput, 0);
	} else if (!lineBreak3Broken && (flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor) < flywheelStoppedRPM) {
		// make sure flywheel is not spinning and we aren't out of room
		ignoreCountingIn = false;
		belt.Set(ControlMode::PercentOutput, forwardBeltSpeed);
	}
		if (LinebreakIn.rising_edge(lineBreak2Broken) && !ignoreCountingIn) {
		count++;
	} else if (LinebreakOut.rising_edge(lineBreak3Broken) && !ignoreCountingOut) {
		count--;
	}
	SmartDashboard::PutNumber("Current Number of balls", count);

}
void Robot::HandleElevator() {
	double encoder = elevatorMotor.GetSelectedSensorPosition();
	debugTab->PutNumber("elevator encoder", encoder);
	bool down = ApplyDeadzone(pilot.GetTriggerAxis(LEFT), prefs.GetDouble("deadzone")) > 0;
	bool up = ApplyDeadzone(pilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool scaryReverse = pilot.GetBackButton();
	if (encoder > minElevatorDown) {
		inputScale = 0.5; // slow down driving when completely extended
	}
	if (up && (encoder < maxElevatorUp) && (encoder < elevatorBreaksPoint)) {
		elevatorMotor.Set(ControlMode::PercentOutput, elevatorSpeedUp);
	} else if (down && (encoder > minElevatorDown) && (encoder < elevatorBreaksPoint)) {
		elevatorMotor.Set(ControlMode::PercentOutput, elevatorSpeedDown);	
	} else if (scaryReverse) {
		elevatorMotor.Set(ControlMode::PercentOutput, -elevatorSpeedDown);
	} else {
		elevatorMotor.Set(ControlMode::PercentOutput, 0);
	}


}
void Robot::HandleIntake(){
	bool isIntaking = ApplyDeadzone(copilot.GetTriggerAxis(LEFT), 0.2) > 0;
	bool isOuttaking = copilot.GetXButton();
	intakePiston.Set(isIntaking || isOuttaking);
	ignoreCountingIn = false;
	if (isOuttaking) { // give outtake priority
		intakeMotor.Set(ControlMode::PercentOutput, -0.85);
		ignoreCountingIn = true;
	} else if (isIntaking) {
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
	} else {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
	}
}
/*==========================
 ___  led strip, color wheel
|_  | ======================
|_  | MORE FUNCTIONALITY ===
|___| ======================
==== vision, playback, paths
============================*/
void Robot::HandleLEDStrip() {
	double angle = copilot.GetPOV();
	bool ledModeDown = angle >= 45 && angle <= 135;
	bool ledModeUp = angle >= 225 && angle <= 315;
	if (ledUp.rising_edge(ledModeUp)) {
		ledMode--;
	} else if (ledDown.rising_edge(ledModeDown)) {
		ledMode++;
	}
	ledStrip.Set(ledMode % ledStrip.NumModes());

	// Output useful values
	debugTab->PutNumber("Led Timer", ledStrip.getTime());
	debugTab->PutNumber("LED MODE", ledMode);
	SmartDashboard::PutString("current LED mode", ledStrip.Get());
}


void Robot::HandleColorWheel() {
	// std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	// char closestColor;
	// double confidence;
	// std::tie(closestColor, confidence) = ClosestColor(colorSensor);
	// int proximity = (int) colorSensor.GetProximity();
	// debugTab->PutNumber("Proximity", proximity);
	// debugTab->PutNumber("Confidence", confidence);
	// debugTab->PutString("Closest Color", std::string(1, closestColor));
	// if(gameData.length() > 0) {
	// 	currentColorTarget = gameData[0];
	// }
	// if ((currentColorTarget == closestColor) || closestColor == 'N' || currentColorTarget == 'N') {
	// 	colorWheelMotor.Set(ControlMode::PercentOutput, 0);
	// } else {
	// 	colorWheelMotor.Set(ControlMode::PercentOutput, -colorSpinnerSpeed);
	// }
	if (copilot.GetYButton()) {
		colorwheelPiston.Set(true);
		colorwheelMotor.Set(ControlMode::PercentOutput, 0.5);
	} else {
		colorwheelMotor.Set(ControlMode::PercentOutput, 0);
		colorwheelPiston.Set(false);
	}

}
void Robot::HandleVision() {
	if (pilot.GetBumperPressed(LEFT)) {
		frontCamera = true;
	} else if (pilot.GetBumperPressed(RIGHT)) { 
		frontCamera = false;
	}
	visionTab->PutBoolean("Front Camera", frontCamera); // frontCamera is balls
	if (pilot.GetAButton()) {
		double target = (frontCamera) ? visionTab->GetNumber("centerXball", centerCamera) : visionTab->GetNumber("centerXshooter", centerCamera);
		double error = ErrorBetween(target, centerCamera);
		// maybe between 0.1 and 0.3
		double turningSpeed = 0.1 + (error * 0.2);
		visionTab->PutNumber("error", error);
		if (target < centerCamera) {
			isAutoAligning = true;
			// drivetrain.ArcadeDrive(0, turningSpeed, false);
		} else if (target > centerCamera) {
			isAutoAligning = true;
			// drivetrain.ArcadeDrive(0, -turningSpeed, false);
		} else { // is now aligned
			// drivetrain.ArcadeDrive(0,0,false);
		}
	} else {
		isAutoAligning = false;
	}
}
void Robot::AutonIntakeAndShoot(std::string trenchPath, std::string shootPath) {
	debugTab->PutNumber("current stage", stage);
	if (stage == 0) {
		intakePiston.Set(true);
		intakeMotor.Set(ControlMode::PercentOutput, 0.5);
	} else if (stage == 1) {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
		intakePiston.Set(false);
		flywheelMotor.Set(ControlMode::Velocity, (int) (flywheelRPM / kTalonRPMConversionFactor));
	} else if (stage == 2) {
		belt.Set(ControlMode::Velocity, beltFireTicks);
	}
	// TODO: THIS DOES NOT WORK RIGHT!!!!
	if (stage == 0) {
		stage = 1;
	} else if (stage == 1) {
		stage = 2;
	}
	// Make sure to feed!!
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
