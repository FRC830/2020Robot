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
	ConfigurePIDF(flywheelMotor, .02, 6E-05, 0, 0);
	flywheelMotor.ConfigClosedloopRamp(2);
	flywheelMotor.SetInverted(true);
	flywheelMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
	SmartDashboard::PutNumber("FLYWHEEL SPEED",flywheelSpeedVelocity);

	// Configure elevator
	elevatorMotor.SetInverted(true);
	elevatorMotor.SetSelectedSensorPosition(0);

	// configure intake
	ConfigurePIDF(intakeBelt, .03, 6E-05, 0, 0);
	intakeBelt.SetInverted(false);
	intakeBelt.SetNeutralMode(motorcontrol::NeutralMode::Brake);
	intakeMotor.SetInverted(true);
	SmartDashboard::PutNumber("INTAKE BELT SHOOT",intakeBeltShootVelocity);

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
  	autonChooser.SetDefaultOption(defaultAuton, defaultAuton);
	autonChooser.AddOption(pathAuton, pathAuton);
	autonChooser.AddOption(defaultAuton, defaultAuton);
	autonChooser.AddOption(simpleAuton, simpleAuton);
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
	frc::SmartDashboard::PutNumber("timer", ledStrip.getTime());
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
	LLead.ResetEncoder();
	RLead.ResetEncoder();
	gyro.Reset();
	ConfigurePIDF(LLeadPID, 0.001,0,0,0);
	ConfigurePIDF(RLeadPID, 0.001,0,0,0);
	TimeFromStart.Reset();
	TimeFromStart.Start();

	// initializes and sets up au
	trajectory = LoadTrajectory("Short.wpilib.json"); 
	frc::Transform2d transform = odometry.GetPose() - trajectory.InitialPose();
	trajectory = trajectory.TransformBy(transform);
	
	//odometry.ResetPosition(trajectory.InitialPose, gyro.GetAngle());


}
void Robot::HandlePathweaver() {

	units::degree_t currentGyroAngle;
	units::meters_per_second_t left;
	units::meters_per_second_t right;

	if(reversedpath){
		currentGyroAngle = units::degree_t(-gyro.GetAngle()+180); // negated so that is clockwise negative
	}else{
		currentGyroAngle = units::degree_t(-gyro.GetAngle()); // negated so that is clockwise negative
	}

	Pose2d currentRobotPose = odometry.Update(units::radian_t(currentGyroAngle), LLead.GetDistance(), RLead.GetDistance());
	const Trajectory::State goal = trajectory.Sample(units::second_t(TimeFromStart.Get())); 
  	ChassisSpeeds adjustedSpeeds = controller.Calculate(currentRobotPose, goal);
	DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.ToWheelSpeeds(adjustedSpeeds);
	// driving backwards should need l and r reversed along with reversed input to motors
	if(reversedpath){
		left = -wheelSpeeds.right;
		right = -wheelSpeeds.left;
	} else {
		left = wheelSpeeds.left;
		right = wheelSpeeds.right;
	}
/*
	if(units::second_t(TimeFromStart.Get()) > trajectory.TotalTime()){
		double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0), flywheelSpeedVelocity);
		if (error <= .1) {
			//intakeBelt.Set(ControlMode::Velocity, intakeBeltShootVelocity);
		}
		trajectory = LoadTrajectory("ballpickup.wpilib.json"); 
		reversedpath = true;
		//resets timer
		TimeFromStart.Reset();
	}
	*/
	// https://docs.wpilib.org/en/latest/docs/software/advanced-control/trajectories/troubleshooting.html
	SmartDashboard::PutNumber("odometry angle", double(units::radian_t(currentGyroAngle)));
	SmartDashboard::PutNumber("left distance", double(LLead.GetDistance()));
	SmartDashboard::PutNumber("right distance", double(RLead.GetDistance()));
	// odometry.Update(units::degree_t(currentGyroAngle),
    //                 units::meter_t(LLead.GetVelocity()),
    //                 units::meter_t(RLead.GetVelocity()));
	SmartDashboard::PutNumber("time from start",TimeFromStart.Get());
	SmartDashboard::PutNumber("adjusted (omega)", (double) adjustedSpeeds.omega);
	SmartDashboard::PutNumber("adjusted (vx)", (double) adjustedSpeeds.vx);
	SmartDashboard::PutNumber("adjusted (vy)", (double) adjustedSpeeds.vy);

	SmartDashboard::PutNumber("gyro (degrees)", (double) currentGyroAngle);
	SmartDashboard::PutNumber("left speed (set) MPS", (double) left);
	SmartDashboard::PutNumber("right speed (set) MPS", (double) right);

	// drivetrain.TankDrive(left,right,false);
	LLead.SetSpeed(0.5*-left);
	RLead.SetSpeed(0.5*right);
	drivetrain.Feed();
}

void Robot::AutonomousPeriodic() {
	//flywheelSpeedVelocity = SmartDashboard::GetNumber("FLYWHEEL SPEED",flywheelSpeedVelocity);

	std::string currentAutonMode = autonChooser.GetSelected();
	if (currentAutonMode == defaultAuton) {
		// do nothing
	} else if (currentAutonMode == simpleAuton) {
		if (LLead.GetDistance() < units::inch_t(48)) {
			drivetrain.ArcadeDrive(0.5, 0, true);
		} else if (LLead.GetDistance() > units::inch_t(48+5)) {
			drivetrain.ArcadeDrive(-0.5, 0, true);
		} else {
			drivetrain.ArcadeDrive(0, 0, true);
		}
	} else if (currentAutonMode == pathAuton) {
	// spin flywheel
	flywheelMotor.Set(TalonFXControlMode::PercentOutput, flywheelSpeedVelocity);
		HandlePathweaver();
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
		if (target < centerCamera) {
			isAutoAligning = true;
			drivetrain.ArcadeDrive(0, 0.1, true);
		} else if (target > centerCamera) {
			isAutoAligning = true;
			drivetrain.ArcadeDrive(0, -0.1, true);
		} else { // bad read, whatever

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
		intakeBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
		return;
	}
	// Start: The 'run forward manually'
	if (copilot.GetStartButton()){
		intakeBelt.Set(ControlMode::PercentOutput, forwardBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
		return;
	}
	
	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0), flywheelSpeedVelocity);
	bool meetsThreshold = (error <= .1);
	bool runFlywheel = ApplyDeadzone(copilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool runShooter = copilot.GetAButton();
	flywheelSpeedVelocity = SmartDashboard::GetNumber("FLYWHEEL SPEED",flywheelSpeedVelocity);
	intakeBeltShootVelocity = SmartDashboard::GetNumber("INTAKE BELT",intakeBeltShootVelocity);
	// Log variables
	SmartDashboard::PutBoolean("Sensor 1 Broken", lineBreak1Broken);
	SmartDashboard::PutBoolean("Sensor 2 Broken", lineBreak2Broken);
	SmartDashboard::PutBoolean("Sensor 3 Broken", lineBreak3Broken);
	SmartDashboard::PutNumber("current intake velocity", intakeBelt.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("current flywheel velocity", flywheelMotor.GetSelectedSensorVelocity(0));
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
	if (runFlywheel || (runShooter && !isUpToSpeed)) {
		if (lineBreak3Broken) { // run belts & flywheel back when touching sensor
			intakeBelt.Set(ControlMode::PercentOutput, -0.85);
			flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelReverseVelocity);
		} else { // otherwise run flywheel and stop running belt back
			flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelSpeedVelocity);
			intakeBelt.Set(ControlMode::PercentOutput, 0);
		}
		return;
	} else if (runShooter && isUpToSpeed) { // fire!
		flywheelMotor.Set(TalonFXControlMode::Velocity, flywheelSpeedVelocity);
		intakeBelt.Set(ControlMode::Velocity, intakeBeltShootVelocity);
		return;
	} else { // not spinning or shooting, so stop flywheel
		flywheelMotor.Set(TalonFXControlMode::PercentOutput, 0);
	}

	// The 'intake' functionality
	if (!lineBreak1Broken) {
		intakeBelt.Set(ControlMode::PercentOutput, 0);
	} else if (!lineBreak3Broken && flywheelMotor.GetSelectedSensorVelocity(0) < flywheelStoppedVelocity) {
		// make sure flywheel is not spinning and we aren't out of room
		intakeBelt.Set(ControlMode::PercentOutput, 0.9);
	}

}
void Robot::HandleElevator() {
	double encoder = elevatorMotor.GetSelectedSensorPosition();
	frc::SmartDashboard::PutNumber("encoder position", encoder);
	bool down = ApplyDeadzone(pilot.GetTriggerAxis(LEFT), prefs.GetDouble("deadzone")) > 0;
	bool up = ApplyDeadzone(pilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	bool scaryReverse = pilot.GetBackButton();

	if(((up && encoder < maxElevatorUp) || (down && encoder > minElevatorDown)) && encoder < elevatorBreaksPoint){
		elevatorMotor.Set(ControlMode::PercentOutput, elevatorSpeed);
	} else if (scaryReverse) {
		elevatorMotor.Set(ControlMode::PercentOutput, -elevatorSpeed);
	} else {
		elevatorMotor.Set(ControlMode::PercentOutput, 0);
	}

}

void Robot::HandleIntake(){
	bool isIntaking = ApplyDeadzone(copilot.GetTriggerAxis(LEFT), 0.2) > 0;
	bool isOuttaking = copilot.GetXButton();
	intakePiston.Set(isIntaking || isOuttaking);

	if (isOuttaking) { // give outtake priority
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
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
