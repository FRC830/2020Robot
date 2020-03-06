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
	LLeadMotor.RestoreFactoryDefaults();
	LFollowMotor.RestoreFactoryDefaults();
	RLeadMotor.RestoreFactoryDefaults();
	RFollowMotor.RestoreFactoryDefaults();
	LFollowMotor.Follow(LLeadMotor, false);
	RFollowMotor.Follow(RLeadMotor, false);
	RLeadMotor.BurnFlash();
	RFollowMotor.BurnFlash();
	LLeadMotor.BurnFlash();
	LFollowMotor.BurnFlash();
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
  	frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

}

void Robot::AutonomousInit() {
	LLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	LFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	RLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	RFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	drivetrain.SetSafetyEnabled(false);
	ConfigurePIDF(LLeadPID, 0.001,0,0,0);
	ConfigurePIDF(RLeadPID, 0.001,0,0,0);
	LLead.ResetEncoder();
	RLead.ResetEncoder();
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
	PlayingBack = false;
	runsAfterPlayback = 5;
	//drivetrain.SetSafetyEnabled(true);

	// Make sure it is back in coast mode
	LLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	LFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	RLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	RFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
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
	// HandleRecordPlayback(); // breaks elevator
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

		flywheelMotor.Set(TalonFXControlMode::Velocity, 10500);
		debugTab->PutNumber("current distance", (double) units::inch_t(LLead.GetDistance()));
		if (RLead.GetDistance() < units::inch_t(115) && BasicTimer.Get() < 3) {
			drivetrain.ArcadeDrive(-0.5, 0, false);
		} else if (!timerStarted) {
			timerStarted = true;
			BasicTimer.Reset();
			BasicTimer.Start();
		} else if (timerStarted && BasicTimer.Get() < 3) {
			drivetrain.ArcadeDrive(0,0,false);
			belt.Set(ControlMode::Velocity, beltFireTicks);
		} else if (BasicTimer.Get() > 3) {
			belt.Set(ControlMode::PercentOutput, 0);
			flywheelMotor.Set(TalonFXControlMode::Velocity, 0);
			if (RLead.GetDistance() > units::inch_t(-60)) {
				drivetrain.ArcadeDrive(0.5,0,false);
			} else {
				drivetrain.ArcadeDrive(0,0,false);
			}
			BasicTimer.Stop();
		}
	} else if (currentAutonMode == simpleAuton) {
		if(SimpleTimeFromStart.Get() < 2){
			drivetrain.ArcadeDrive(-0.5,0,false);
		} else {
			drivetrain.ArcadeDrive(0,0,false);
		}
		
	} else if (currentAutonMode == straightPathAuton) {
		//AutonIntakeAndShoot("straight","straight");
	} else if (currentAutonMode == middlePathAuton) {
	// spin flywheel
		AutonIntakeAndShoot("Unnamed","Unnamed_0");
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

	double speed = ApplyDeadzone(pilot.GetY(LEFT), prefs.GetDouble("deadzone"));
	double turn = ApplyDeadzone(pilot.GetX(RIGHT), prefs.GetDouble("deadzone"));

	if(!PlayingBack && !isAutoAligning) {
		drivetrain.ArcadeDrive(speed * .65 * inputScale, -(turn * .75), true);
		
	}

	// Output useful values
	debugTab->PutNumber("Yaw",gyro.GetYaw());
	debugTab->PutNumber("Pitch",gyro.GetPitch());
	debugTab->PutNumber("Roll",gyro.GetRoll());
	debugTab->PutNumber("Turn", turn);
	debugTab->PutNumber("left lead position", LLead.GetPosition());
	debugTab->PutNumber("right lead position", RLead.GetPosition());
	debugTab->PutNumber("Current L motor velocity", LLead.GetVelocity());
	debugTab->PutNumber("Current R motor velocity", RLead.GetVelocity());
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
		double intakeSpeed = (manualBeltPower > 0) ? intakeRollerSpeed : -intakeRollerSpeed;
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
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
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
			drivetrain.ArcadeDrive(0, turningSpeed, false);
		} else if (target > centerCamera) {
			isAutoAligning = true;
			drivetrain.ArcadeDrive(0, -turningSpeed, false);
		} else { // is now aligned
			drivetrain.ArcadeDrive(0,0,false);
		}
	} else {
		isAutoAligning = false;
	}
}
void Robot::HandleRecordPlayback() {
	if(pilot.GetStartButtonPressed()) {
		recordGo = false;
		isRecording = !isRecording;
		debugTab->PutBoolean("is recording", isRecording);
		LLeadMotor.GetEncoder().SetPosition(0.0);
		LFollowMotor.GetEncoder().SetPosition(0.0);
		RLeadMotor.GetEncoder().SetPosition(0.0);
		RFollowMotor.GetEncoder().SetPosition(0.0);
		if(!isRecording) { // recording has just finished, save to a file
			outputToFile({leftLeadMotorValues, leftFollowMotorValues, rightFollowMotorValues, rightLeadMotorValues}, "/home/lvuser/vectors.txt");
		}else { // wipe the array, recording started
			leftLeadMotorValues.clear();
			leftFollowMotorValues.clear();
			rightLeadMotorValues.clear();
			rightFollowMotorValues.clear();
		}
	}
	
	bool allEncodersZero = (LLeadMotor.GetEncoder().GetPosition() == 0.0 && 
	RLeadMotor.GetEncoder().GetPosition() == 0.0 &&
	LFollowMotor.GetEncoder().GetPosition() == 0.0 &&
	RFollowMotor.GetEncoder().GetPosition() == 0.0);
	//allEd
	debugTab->PutBoolean("all encoders are zero", allEncodersZero);

	if (allEncodersZero) {
		recordGo = true;
	}
	// make sure that encoders are wiped before pushing data
	if(isRecording && recordGo){
		leftLeadMotorValues.push_back(LLead.GetPosition());
		leftFollowMotorValues.push_back(LFollow.GetPosition());
		rightLeadMotorValues.push_back(RLead.GetPosition());
		rightFollowMotorValues.push_back(RFollow.GetPosition());
	}

	// start playback
	if(pilot.GetBButtonPressed() || pilot.GetBButtonReleased()) {
		LLeadMotor.GetEncoder().SetPosition(0.0);
		LFollowMotor.GetEncoder().SetPosition(0.0);
		RLeadMotor.GetEncoder().SetPosition(0.0);
		RFollowMotor.GetEncoder().SetPosition(0.0); // returns in 1ms, motor doesnt actually set till 50ms
		
		PlayingBack = !PlayingBack;
		debugTab->PutBoolean("is playing back", PlayingBack);
		runsAfterPlayback = 0;
		pilot.SetRumble(GenericHID::kLeftRumble, 0);
		pilot.SetRumble(GenericHID::kRightRumble, 0);

		if (PlayingBack) {
			SetPID(LLeadMotor, kPposi, kIposi, kDposi);
			SetPID(RLeadMotor, kPposi, kIposi, kDposi);
			SetPID(RFollowMotor, kPposi, kIposi, kDposi);
			SetPID(LFollowMotor, kPposi, kIposi, kDposi);
		}
	}
	// playback
	if (PlayingBack) {
	
		LLeadMotor.GetPIDController().SetReference(leftLeadMotorValues.at(runsAfterPlayback), rev::ControlType::kPosition);
		LFollowMotor.GetPIDController().SetReference(leftFollowMotorValues.at(runsAfterPlayback), rev::ControlType::kPosition);
		RLeadMotor.GetPIDController().SetReference(rightLeadMotorValues.at(runsAfterPlayback), rev::ControlType::kPosition);
		RFollowMotor.GetPIDController().SetReference(rightFollowMotorValues.at(runsAfterPlayback), rev::ControlType::kPosition);

		pilot.SetRumble(GenericHID::kLeftRumble, 1);
		pilot.SetRumble(GenericHID::kRightRumble, 1);

		runsAfterPlayback++;
		if (leftLeadMotorValues.size() <= runsAfterPlayback) {	
			PlayingBack = false;	
			pilot.SetRumble(GenericHID::kLeftRumble, 0);	
			pilot.SetRumble(GenericHID::kRightRumble, 0);	
		}
	}
}
void Robot::AutonIntakeAndShoot(std::string trenchPath, std::string shootPath) {
	debugTab->PutNumber("current stage", stage);
	debugTab->PutString("current path", pathProcessor.getCurrentPath());
	debugTab->PutNumber("current gyro", pathProcessor.getCurrentAngle());
	if (stage == 0) {
		pathProcessor.runPathUntilFinished(trenchPath, false);
		intakePiston.Set(true);
		intakeMotor.Set(ControlMode::PercentOutput, 0.5);
	} else if (stage == 1) {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
		intakePiston.Set(false);
		flywheelMotor.Set(ControlMode::Velocity, (int) (flywheelRPM / kTalonRPMConversionFactor));
		pathProcessor.runPathUntilFinished(shootPath, true);
	} else if (stage == 2) {
		belt.Set(ControlMode::Velocity, beltFireTicks);
	}
	drivetrain.Feed();
	if (pathProcessor.pathCompleted() && stage == 0) {
		stage = 1;
	} else if (pathProcessor.pathCompleted() && stage == 1) {
		stage = 2;
	}
	// Make sure to feed!!
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
