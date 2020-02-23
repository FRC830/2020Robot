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
	flywheelMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
	frc::ShuffleboardTab &visionTab = frc::Shuffleboard::GetTab("vision");
	frc::SmartDashboard::PutNumber("LED MODE", ledMode);
	frc::SmartDashboard::PutString("current LED mode", ledStrip.Get());
	// configure intake
	ConfigurePIDF(intakeBelt, .03, 6E-05, 0, 0);
	intakeBelt.SetInverted(true);
	intakeMotor.SetInverted(true);

	// Configure Line break sensors & belts
	SmartDashboard::PutBoolean("Line Break Sensor 1", false);
	SmartDashboard::PutBoolean("Line Break Sensor 2", false);
	SmartDashboard::PutBoolean("Line Break Sensor 3", false);
	// shooterBelt.SetInverted(true);
	SmartDashboard::PutNumber("FLYWHEEL SPEED",flywheelSpeedVelocity);
	SmartDashboard::PutNumber("INTAKE BELT SHOOT",intakeBeltShootVelocity);
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

  	autonChooser.SetDefaultOption(defaultAuton, defaultAuton);
 	 autonChooser.AddOption(pathAuton, pathAuton);
 	 autonChooser.AddOption(defaultAuton, defaultAuton);
 	 autonChooser.AddOption(simpleAuton, simpleAuton);
  frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

	RLeadMotor.BurnFlash();
	RFollowMotor.BurnFlash();
	LLeadMotor.BurnFlash();
	LFollowMotor.BurnFlash();
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
	} else {
		/// return;
	}
	frc::SmartDashboard::PutNumber("timer", ledStrip.getTime());
	frc::SmartDashboard::PutNumber("LED MODE", ledMode);
	ledStrip.Set(ledMode % ledStrip.NumModes());
	frc::SmartDashboard::PutString("current LED mode", ledStrip.Get());
}
void Robot::RobotPeriodic() {
	frc::SmartDashboard::PutNumber("Current L motor velocity", LLead.GetVelocity());
	frc::SmartDashboard::PutNumber("Current R motor velocity", RLead.GetVelocity());
}

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

	trajectory = LoadTrajectory("auton1.wpilib.json");
	
	// frc::Transform2d transform = Pose2d(4_m,4_m, Rotation2d(90_deg)) - trajectory.IntialPose();
	//  
	frc::Transform2d transform = odometry.GetPose() - trajectory.InitialPose();
	trajectory = trajectory.TransformBy(transform);
	

}
void Robot::HandlePathweaver() {
	auto currentGyroAngle = units::degree_t(-gyro.GetAngle()); // negated so that is clockwise negative
	// https://docs.wpilib.org/en/latest/docs/software/advanced-control/trajectories/troubleshooting.html
	SmartDashboard::PutNumber("odometry angle", double(units::radian_t(currentGyroAngle)));
	SmartDashboard::PutNumber("left distance", double(LLead.GetDistance()));
	SmartDashboard::PutNumber("right distance", double(RLead.GetDistance()));
	Pose2d currentRobotPose = odometry.Update(units::radian_t(currentGyroAngle), LLead.GetDistance(), RLead.GetDistance());
	const Trajectory::State goal = trajectory.Sample(units::second_t(TimeFromStart.Get())); 
  	ChassisSpeeds adjustedSpeeds = controller.Calculate(currentRobotPose, goal);
	// odometry.Update(units::degree_t(currentGyroAngle),
    //                 units::meter_t(LLead.GetVelocity()),
    //                 units::meter_t(RLead.GetVelocity()));
	SmartDashboard::PutNumber("time from start",TimeFromStart.Get());
	SmartDashboard::PutNumber("adjusted (omega)", (double) adjustedSpeeds.omega);
	SmartDashboard::PutNumber("adjusted (vx)", (double) adjustedSpeeds.vx);
	SmartDashboard::PutNumber("adjusted (vy)", (double) adjustedSpeeds.vy);
	DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.ToWheelSpeeds(adjustedSpeeds);
	units::meters_per_second_t left = wheelSpeeds.left;
	units::meters_per_second_t right = wheelSpeeds.right;
	SmartDashboard::PutNumber("gyro (degrees)", (double) currentGyroAngle);
	SmartDashboard::PutNumber("left speed (set) MPS", (double) left);
	SmartDashboard::PutNumber("right speed (set) MPS", (double) right);

	// drivetrain.TankDrive(left,right,false);
	LLead.SetSpeed(0.5*-left);
	RLead.SetSpeed(0.5*right);
}
void Robot::AutonomousPeriodic() {
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
		HandlePathweaver();
	}
}

void Robot::TeleopInit() {}

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
	// HandleRecordPlayback();
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
	
	// Start: The 'run in reverse because something bad happened'
	if (copilot.GetBackButton()) {
		// shooterBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
		return; // do not run any other shooter code
	}
	// Back: The 'run forward manually'
	if (copilot.GetStartButton()){
		intakeBelt.Set(ControlMode::PercentOutput, forwardBeltSpeed);
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
		return; // do not run any other shooter code
		// if (!lineBreak2Broken){
		// 	// shooterBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		// 	intakeBelt.Set(ControlMode::PercentOutput, -reverseBeltSpeed);
		// 	intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);

		// } else {
		// 	// shooterBelt.Set(ControlMode::PercentOutput, 0);
		// 	intakeBelt.Set(ControlMode::PercentOutput, 0);
		// }
		// // already handled sensor here, prevent it from being handled later
		// lineBreak2WasBroken = lineBreak2Broken;
	}
	// The 'spin flywheel' functionality
	double error = ErrorBetween(flywheelMotor.GetSelectedSensorVelocity(0), flywheelSpeedVelocity);
	bool meetsThreshold = (error <= .1);
	
	bool runFlywheel = ApplyDeadzone(copilot.GetTriggerAxis(RIGHT), prefs.GetDouble("deadzone")) > 0;
	flywheelSpeedVelocity = SmartDashboard::GetNumber("FLYWHEEL SPEED",flywheelSpeedVelocity);
	intakeBeltShootVelocity = SmartDashboard::GetNumber("INTAKE BELT",intakeBeltShootVelocity);
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
			// shooterBelt.Set(ControlMode::PercentOutput, shooterBeltSpeed);
			intakeBelt.Set(ControlMode::Velocity, intakeBeltShootVelocity);
		}
		return; // do not run intake code
	}

	// The 'intake' functionality
	if (!lineBreak1Broken && !lineBreak2Broken) {
		intakeBelt.Set(ControlMode::PercentOutput, 0);
		// shooterBelt.Set(ControlMode::PercentOutput, 0);
	} else {
		intakeBelt.Set(ControlMode::Velocity, intakeBeltSpeedVelocity);
		// shooterBelt.Set(ControlMode::PercentOutput, shooterBeltSpeed);
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
	bool isIntaking = ApplyDeadzone(copilot.GetTriggerAxis(LEFT), 0.2) > 0;
	bool isOuttaking = copilot.GetXButton();
	frc::SmartDashboard::PutBoolean("is intaking",isIntaking);
	intakePiston.Set(isIntaking);
	if (isIntaking){
		intakeMotor.Set(ControlMode::PercentOutput, intakeRollerSpeed);
	} else if (isOuttaking) {
		intakeMotor.Set(ControlMode::PercentOutput, -intakeRollerSpeed);
	} else {
		intakeMotor.Set(ControlMode::PercentOutput, 0);
	}
}

void Robot::TestPeriodic() {}

void Robot::DisabledInit() {
	PlayingBack = false;
	runsAfterPlayback = 5;
	drivetrain.SetSafetyEnabled(true);
	LLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	LFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	RLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	RFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
