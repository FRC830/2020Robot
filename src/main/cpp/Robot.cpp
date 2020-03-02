#include "Robot.h"																														

using namespace frc;
void Robot::RobotInit() {
	// Configure drivetrain
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

	// Configure flywheel
	ConfigurePIDF(flywheelMotor, .05, 6E-05, 0, 0);
	flywheelMotorFollow.Follow(flywheelMotor);
	flywheelMotorFollow.SetInverted(false);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(true);
	flywheelMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
	SmartDashboard::PutNumber("FLYWHEEL RPM",flywheelRPM);
	//gyro.Calibrate();
	// Configure elevator
	elevatorMotor.SetInverted(true);
	elevatorMotor.SetSelectedSensorPosition(0);

	// configure intake
	ConfigurePIDF(belt, .03, 6E-05, 0, 0);
	belt.SetInverted(false);
	belt.SetNeutralMode(motorcontrol::NeutralMode::Brake);
	intakeMotor.SetInverted(true);
	SmartDashboard::PutNumber("INTAKE BELT SHOOT",beltFireTicks);

	// Configure LED Strip
	frc::SmartDashboard::PutNumber("LED MODE", ledMode);
	frc::SmartDashboard::PutString("current LED mode", ledStrip.Get());

	// Configure Line break sensors & belts
	SmartDashboard::PutBoolean("Line Break Sensor 1", false);
	SmartDashboard::PutBoolean("Line Break Sensor 2", false);
	SmartDashboard::PutBoolean("Line Break Sensor 3", false);

	// Configure Vision
	frc::ShuffleboardTab &visionTab = frc::Shuffleboard::GetTab("vision");
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

	// Configure auton
  	autonChooser.SetDefaultOption(simpleAuton, simpleAuton);
	autonChooser.AddOption(middlePathAuton, middlePathAuton); // starts from center of field
	autonChooser.AddOption(straightPathAuton, straightPathAuton); // starts from right side of field, but driver sees it as left
	autonChooser.AddOption(noneAuton, noneAuton); // do nothing
	autonChooser.AddOption(basicAuton, basicAuton); // go forward, shoot, go back
	autonChooser.AddOption(simpleAuton, simpleAuton); // very simple, just get off line
  	frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

}

// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {

	double speed = ApplyDeadzone(pilot.GetY(LEFT), prefs.GetDouble("deadzone"));
	double turn = ApplyDeadzone(pilot.GetX(RIGHT), prefs.GetDouble("deadzone"));

	if(!PlayingBack && !isAutoAligning) {
		drivetrain.ArcadeDrive(speed, -turn, true);
		
	}

	// Output useful values
	frc::SmartDashboard::PutNumber("Yaw",gyro.GetYaw());
	frc::SmartDashboard::PutNumber("Pitch",gyro.GetPitch());
	frc::SmartDashboard::PutNumber("Roll",gyro.GetRoll());
	frc::SmartDashboard::PutNumber("Turn", turn);
	frc::SmartDashboard::PutNumber("left lead position", LLead.GetPosition());
	frc::SmartDashboard::PutNumber("right lead position", RLead.GetPosition());
	frc::SmartDashboard::PutNumber("Current L motor velocity", LLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Current R motor velocity", RLead.GetVelocity());
}
// Handle LED Strip code
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
	frc::SmartDashboard::PutNumber("Led Timer", ledStrip.getTime());
	frc::SmartDashboard::PutNumber("LED MODE", ledMode);
	frc::SmartDashboard::PutString("current LED mode", ledStrip.Get());
}
void Robot::RobotPeriodic() {}

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
void Robot::AutonIntakeAndShoot(std::string trenchPath, std::string shootPath) {
	SmartDashboard::PutNumber("current stage", stage);
	SmartDashboard::PutString("current path", pathProcessor.getCurrentPath());
	SmartDashboard::PutNumber("current gyro", pathProcessor.getCurrentAngle());
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
	// SmartDashboard::PutNumber("odometry angle", double(units::radian_t(currentGyroAngle)));
	// SmartDashboard::PutNumber("left distance", double(LLead.GetDistance()));
	// SmartDashboard::PutNumber("right distance", double(RLead.GetDistance()));
	// // odometry.Update(units::degree_t(currentGyroAngle),
    // //                 units::meter_t(LLead.GetVelocity()),
    // //                 units::meter_t(RLead.GetVelocity()));
	// SmartDashboard::PutNumber("time from start",TimeFromStart.Get());
	// SmartDashboard::PutNumber("adjusted (omega)", (double) adjustedSpeeds.omega);
	// SmartDashboard::PutNumber("adjusted (vx)", (double) adjustedSpeeds.vx);
	// SmartDashboard::PutNumber("adjusted (vy)", (double) adjustedSpeeds.vy);

	// SmartDashboard::PutNumber("gyro (degrees)", (double) currentGyroAngle);
	// SmartDashboard::PutNumber("left speed (set) MPS", (double) left);
	// SmartDashboard::PutNumber("right speed (set) MPS", (double) right);
	// Make sure to feed!!
}
void Robot::AutonomousPeriodic() {
	//flywheelRPM = SmartDashboard::GetNumber("FLYWHEEL SPEED",flywheelRPM);
	std::string currentAutonMode = autonChooser.GetSelected();
	if (currentAutonMode == noneAuton) {
		// do nothing
	} else if (currentAutonMode == basicAuton) {

		flywheelMotor.Set(TalonFXControlMode::Velocity, 10500);
		SmartDashboard::PutNumber("current distance", (double) units::inch_t(LLead.GetDistance()));
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

void Robot::TeleopInit() {
	// reset elevator
	elevatorMotor.SetSelectedSensorPosition(0);
}

void Robot::HandleVision() {
	if (pilot.GetBumperPressed(LEFT)) {
		frontCamera = true;
	} else if (pilot.GetBumperPressed(RIGHT)) { 
		frontCamera = false;
	}
	visionTab2->PutBoolean("Front Camera", frontCamera); // frontCamera is balls
	if (pilot.GetAButton()) {
		double target = (frontCamera) ? visionTab2->GetNumber("centerXball", centerCamera) : visionTab2->GetNumber("centerXshooter", centerCamera);
		double error = ErrorBetween(target, centerCamera);
		// maybe between 0.1 and 0.3
		double turningSpeed = 0.1 + (error * 0.2);
		visionTab2->PutNumber("error", error);
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
		SmartDashboard::PutBoolean("is recording", isRecording);
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
	SmartDashboard::PutBoolean("all encoders are zero", allEncodersZero);

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
		SmartDashboard::PutBoolean("is playing back", PlayingBack);
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

void Robot::TeleopPeriodic() {
	HandleLEDStrip();
	HandleDrivetrain();
	// HandleRecordPlayback(); // breaks elevator
	// HandleColorWheel(); // Currently breaks robot code w/o sensor
	HandleIntake();
	HandleShooter();
	HandleVision();
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
	bool lineBreak1Broken = !lineBreak1.Get();
	bool lineBreak2Broken = !lineBreak2.Get();
	bool lineBreak3Broken = !lineBreak3.Get();
	
	// Back: The 'run in reverse manually'
	if (copilot.GetBackButton()) {
		belt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
		return;
	}
	// Start: The 'run forward manually'
	if (copilot.GetStartButton()){
		belt.Set(ControlMode::PercentOutput, forwardBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
		return;
	}

	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor, flywheelRPM);
	bool meetsThreshold = (error <= .1);
	bool runFlywheel = ApplyDeadzone(copilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool runShooter = copilot.GetAButton();
	flywheelRPM = SmartDashboard::GetNumber("FLYWHEEL RPM",flywheelRPM);
	beltFireTicks = SmartDashboard::GetNumber("INTAKE BELT",beltFireTicks);
	// Log variables
	SmartDashboard::PutBoolean("Sensor 1 Broken", lineBreak1Broken);
	SmartDashboard::PutBoolean("Sensor 2 Broken", lineBreak2Broken);
	SmartDashboard::PutBoolean("Sensor 3 Broken", lineBreak3Broken);
	SmartDashboard::PutNumber("current intake velocity", belt.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("current flywheel velocity", flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor);
	SmartDashboard::PutNumber("current error", std::fabs(flywheelMotor.GetClosedLoopError(0)));
	// Update the status of up to speed
	if ((runShooter || runFlywheel) && meetsThreshold) {
		isUpToSpeed = true;
	} else if (!meetsThreshold) { // Does not check this if we are shooting/running flywheel and it was up to speed at one point
		isUpToSpeed = false;
	}
	frc::SmartDashboard::PutBoolean("meets threshold", meetsThreshold);
	frc::SmartDashboard::PutBoolean("is up to speed", isUpToSpeed);
	// The 'spin flywheel && shoot' functionality
	if (runShooter && isUpToSpeed) { // fire!
		flywheelMotor.Set(TalonFXControlMode::Velocity, (int) (flywheelRPM / kTalonRPMConversionFactor));
		belt.Set(ControlMode::Velocity, beltFireTicks);
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
		belt.Set(ControlMode::PercentOutput, 0);
	} else if (!lineBreak3Broken && (flywheelMotor.GetSelectedSensorVelocity(0) * kTalonRPMConversionFactor) < flywheelStoppedRPM) {
		// make sure flywheel is not spinning and we aren't out of room
		belt.Set(ControlMode::PercentOutput, forwardBeltSpeed);
	}

}
void Robot::HandleElevator() {
	double encoder = elevatorMotor.GetSelectedSensorPosition();
	frc::SmartDashboard::PutNumber("encoder position", encoder);
	bool down = ApplyDeadzone(pilot.GetTriggerAxis(LEFT), prefs.GetDouble("deadzone")) > 0;
	bool up = ApplyDeadzone(pilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool scaryReverse = pilot.GetBackButton();

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

	if (isOuttaking) { // give outtake priority
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
	} else if (isIntaking) {
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
	} else {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
	}
}

void Robot::TestPeriodic() {}

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

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
