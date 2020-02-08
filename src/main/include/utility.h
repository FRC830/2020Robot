#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/util/Color.h>
#include "ctre/Phoenix.h"
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SmartDashboard.h>

// apply deadzone & possible scaling, etc
double ApplyDeadzone(double val, double deadzone) {
	return fabs(val) < deadzone ? 0 : val;
}
// Return the closest detected color
frc::Color aimRed = {0.465, 0.376, 0.158}; 
frc::Color aimYellow = {0.324, 0.535, 0.14}; 
frc::Color aimGreen = {0.197, 0.545, 0.256}; 
frc::Color aimBlue = {0.157, 0.43, 0.412}; 
    rev::ColorMatch colorMatcher;

std::tuple<char, double> ClosestColor(rev::ColorSensorV3 &colorSensor) {
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
// For TalonSRX
void ConfigurePIDF(TalonSRX &motor, double p, double i, double d, double f, bool reset=true) {
    if (reset) {
        motor.ConfigFactoryDefault();
	}
	motor.Config_kP(0, p);
	motor.Config_kI(0, i);
	motor.Config_kD(0, d);
	motor.Config_kF(0, f);
}
// For TalonFX
void ConfigurePIDF(TalonFX &motor, double p, double i, double d, double f, bool reset=true) {
    if (reset) {
        motor.ConfigFactoryDefault();
    }
	motor.Config_kP(0, p);
	motor.Config_kI(0, i);
	motor.Config_kD(0, d);
	motor.Config_kF(0, f);
}
// For Plain PID Controller
void ConfigurePIDF(rev::CANPIDController pid, double p, double i, double d, double f, double range=1.0) {
	pid.SetOutputRange(-range, range);
	pid.SetP(p);
	pid.SetI(i);
	pid.SetD(d);
	pid.SetFF(f);
}

void MakeSlider(frc::ShuffleboardTab &tab, std::string name, double defaultV, double max=255) {
		wpi::StringMap<std::shared_ptr<nt::Value>> properties {
				std::make_pair("min", nt::Value::MakeDouble(0)),
				std::make_pair("max", nt::Value::MakeDouble(max))
		};
		tab
		.Add(name, defaultV)
		.WithWidget(frc::BuiltInWidgets::kNumberSlider)
		.WithProperties(properties);
}

double ErrorBetween(double a, double b) {
	if (a==0||b==0) {
		return 1.0;
	}
	if (a>b) {
		return a/b - 1.0;
	}
	return b/a - 1.0;
}