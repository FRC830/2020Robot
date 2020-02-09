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
		RFollowMotor.Follow(RLeadMotor, true); // differential drive inverts the right motor
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
		prefs.PutInt("shooter output in ticks", 8000);
		
		// configure intake/shooter
		prefs.PutBoolean("wait to shoot", false);
		prefs.PutDouble("intake motor speed", 0.2);
		prefs.PutDouble("intake belt speed", 0.2);
		prefs.PutDouble("shooter belt speed", 0.2);
		prefs.PutDouble("shooter belt speed reverse", 0.2);
		// output vision testing values
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

		kPposi = SmartDashboard::GetNumber("kp", kPposi);
  		kIposi = SmartDashboard::GetNumber("ki", kIposi);
  		kDposi = SmartDashboard::GetNumber("kd", kDposi);
		
}
void Robot::print(std::vector<double> input)
{
	std::ofstream values;

	values.open ("vectors.txt");
	int size = input.size() - 1;
	for (int i = 0; i < size; i++) {
		values << input.at(i) << ',';
	}
	values << "\n";
	values.close();
}
void Robot::printSD(std::vector<double> input, std::string name)
{
	int size = input.size();
	for (int i = 0; i < size; i++) {
		SmartDashboard::PutNumber(std::to_string(i) + name, input.at(i));
	}
}
// Handle teleop drivetrain code
void Robot::HandleDrivetrain() {
	double speed;
	double turn;
	double targetVelocity;
	
	if(!PlayingBack)
	{
		speed = ProcessControllerInput(pilot.GetY(LEFT));
		turn = ProcessControllerInput(pilot.GetX(RIGHT));
		targetVelocity = speed;
		drivetrain.ArcadeDrive(targetVelocity, -turn, true);
		
	}
	else if (PlayingBack)
	{
		const int MOD = runsAfterPlayback - (runsAfterPlayback % 2); 
		LLeadMotor.GetPIDController().SetReference(leftLeadMotorValues.at(MOD), rev::ControlType::kPosition);
		LFollowMotor.GetPIDController().SetReference(leftFollowMotorValues.at(MOD), rev::ControlType::kPosition);
		RLeadMotor.GetPIDController().SetReference(rightLeadMotorValues.at(MOD), rev::ControlType::kPosition);
		RFollowMotor.GetPIDController().SetReference(rightFollowMotorValues.at(MOD), rev::ControlType::kPosition);

		pilot.SetRumble(GenericHID::kLeftRumble, 1);
		pilot.SetRumble(GenericHID::kRightRumble, 1);

		runsAfterPlayback++;
		if (leftLeadMotorValues.size() <= runsAfterPlayback)
		{
			PlayingBack = false;
			pilot.SetRumble(GenericHID::kLeftRumble, 0);
			pilot.SetRumble(GenericHID::kRightRumble, 0);
		}
	}
	// Output useful values
	frc::SmartDashboard::PutNumber("Speed", speed);
	frc::SmartDashboard::PutNumber("Turn", turn);
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
  RLead.UseEncoder(prefs.GetBoolean("use encoder"));
  LLead.UseEncoder(prefs.GetBoolean("use encoder"));
  InitializePIDController(LLeadPID);
  InitializePIDController(RLeadPID);

  kPposi = SmartDashboard::PutNumber("kp", kPposi);
  kIposi = SmartDashboard::PutNumber("ki", kIposi);
  kDposi = SmartDashboard::PutNumber("kd", kDposi);
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

	kPposi = SmartDashboard::GetNumber("kp", kPposi);
  	kIposi = SmartDashboard::GetNumber("ki", kIposi);
    kDposi = SmartDashboard::GetNumber("kd", kDposi);

}
void Robot::HandleStuff() {
	bool isIntaking = ProcessControllerInput(copilot.GetTriggerAxis(LEFT)) > 0;
	bool runShooterBelt = ProcessControllerInput(copilot.GetTriggerAxis(RIGHT)) > 0;
	int error = std::abs(flywheelMotor.GetClosedLoopError());

	canIntake.toggle(copilot.GetXButton());
	isShooting.toggle(copilot.GetAButton());

	SmartDashboard::PutBoolean("isIntaking", isIntaking);
	SmartDashboard::PutBoolean("canIntake", canIntake);
	SmartDashboard::PutBoolean("wants to shoot", isShooting);

	//intakePiston.Set(canIntake);
	intakeMotor.Set(ControlMode::PercentOutput, isIntaking ? prefs.GetDouble("intake belt speed",0) : 0);
	shooterBelt.Set(ControlMode::PercentOutput, isIntaking ? prefs.GetDouble("shooter belt speed",0) : 0);
	// turn on shooter belt
	if (runShooterBelt) {
		shooterBelt.Set(ControlMode::PercentOutput, prefs.GetDouble("shooter belt speed", 0));
	}

	if (isShooting) {
		if (error > prefs.GetInt("flywheel error", 0)) {
			shooterBelt.Set(ControlMode::PercentOutput, isIntaking ? -prefs.GetDouble("shooter belt speed reverse",0) : 0);
		}
		
		flywheelMotor.Set(TalonFXControlMode::Velocity, prefs.GetInt("shooter output in ticks", 0));
	}
	
	//recording/.playback options
	//recording
	if(pilot.GetAButtonPressed()) {
		isRecording = !isRecording;
		SmartDashboard::PutBoolean("is recording", isRecording);
		LLeadMotor.GetEncoder().SetPosition(0.0);
		LFollowMotor.GetEncoder().SetPosition(0.0);
		RLeadMotor.GetEncoder().SetPosition(0.0);
		RFollowMotor.GetEncoder().SetPosition(0.0);
		if(!isRecording){
			print(leftLeadMotorValues);
			print(leftFollowMotorValues);
			print(rightLeadMotorValues);
			print(rightFollowMotorValues);
		}else{
			leftLeadMotorValues.clear();
			leftFollowMotorValues.clear();
			rightLeadMotorValues.clear();
			rightFollowMotorValues.clear();
		}
	}
	
	/*bool allEncodersZero = (LLeadMotor.GetEncoder().GetPosition() == 0.0 && 
							RLeadMotor.GetEncoder().GetPosition() == 0.0 &&
							LFollowMotor.GetEncoder().GetPosition() == 0.0 &&
							RFollowMotor.GetEncoder().GetPosition() == 0.0
						);*/
	//allEd
	SmartDashboard::PutBoolean("all encoders are zero", allEncodersZero);
	if(isRecording && true/*allEncodersZero*/){
		leftLeadMotorValues.push_back(LLead.GetEncoder());
		leftFollowMotorValues.push_back(LFollow.GetEncoder());
		rightLeadMotorValues.push_back(RLead.GetEncoder());
		rightFollowMotorValues.push_back(RFollow.GetEncoder());
	}

	//playback
	if(pilot.GetBButtonPressed())
	{
		LLeadMotor.GetEncoder().SetPosition(0.0);
		LFollowMotor.GetEncoder().SetPosition(0.0);
		RLeadMotor.GetEncoder().SetPosition(0.0);
		RFollowMotor.GetEncoder().SetPosition(0.0); // returns in 1ms, motor doesnt actually set till 50ms
		
		PlayingBack = !PlayingBack;
		SmartDashboard::PutBoolean("is playing back", PlayingBack);
		runsAfterPlayback = 0;
		pilot.SetRumble(GenericHID::kLeftRumble, 0);
		pilot.SetRumble(GenericHID::kRightRumble, 0);

		if (PlayingBack)
		{
			LLeadMotor.GetPIDController().SetP(kPposi);
			LLeadMotor.GetPIDController().SetI(kIposi);
			LLeadMotor.GetPIDController().SetD(kDposi);

			LFollowMotor.GetPIDController().SetP(kPposi);
			LFollowMotor.GetPIDController().SetI(kIposi);
			LFollowMotor.GetPIDController().SetD(kDposi);

			RLeadMotor.GetPIDController().SetP(kPposi);
			RLeadMotor.GetPIDController().SetI(kIposi);
			RLeadMotor.GetPIDController().SetD(kDposi);

			RFollowMotor.GetPIDController().SetP(kPposi);
			RFollowMotor.GetPIDController().SetI(kIposi);
			RFollowMotor.GetPIDController().SetD(kDposi);
		}
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
std::vector<double> leftLeadMotorValues;
std::vector<double> rightLeadMotorValues;
std::vector<double> leftFollowMotorValues;
std::vector<double> rightFollowMotorValues;
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif