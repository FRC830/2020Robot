#define _USE_MATH_DEFINES
#include <math.h>
// #include <cmath>
#include <string>

// #include <SPI.h>

#include <rev/CANSparkMax.h>
#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class SwerveModule {
    private:
    std::string m_name;
    rev::CANSparkMax m_wheel; // wheel motor
    rev::CANSparkMax m_turn; // turn motor
    ctre::phoenix::sensors::CANCoder m_turnCANCoder; // Motor CANCoder
    double m_wheelSpeed;
    double m_desiredAngle;
    const double m_p = 0.05;
    const double m_i = 0.0;
    const double m_d = 0.0; 
    // drive is 6.6:1
    const double m_turnGearRatio = 12.8; // To do a 360, we have to turn the gear 100 times
    void setPID(double p, double i, double d) {
        auto pid = m_turn.GetPIDController();
        pid.SetP(p);
        pid.SetI(i);
        pid.SetD(d);
    }
    public:  
    SwerveModule(std::string name, int wID, int tID, int turnCANCoderID) : 
        m_wheel(wID, rev::CANSparkMax::MotorType::kBrushless), 
        m_turn(tID, rev::CANSparkMax::MotorType::kBrushless),       
        m_turnCANCoder(turnCANCoderID)
    {
        m_name = name;
        setPID(m_p, m_i, m_d);
    }
    void setDesiredAngle(double angle) {
        frc::SmartDashboard::PutNumber(m_name + " Desired Angle: ", angle);
        m_desiredAngle = angle;
    }
    void setWheelSpeed(double ws) {
        m_wheelSpeed = ws;
    }

    void apply() {
        m_wheel.Set(m_wheelSpeed);

        frc::SmartDashboard::PutNumber(m_name + " speed", m_wheelSpeed);

        // Currently have an angle in radians
        // Convert this into "Ticks"
        // [-π, π] => [-1, 1] # [-100, 100], then i*(100/2π) is the current position in ticks
        auto pid = m_turn.GetPIDController();
        //All three below variables are in wheel rotations
        double rawRotations = (m_turn.GetEncoder().GetPosition()) / m_turnGearRatio; // 5.75
        frc::SmartDashboard::PutNumber(m_name + " Current Set Point", rawRotations); 
        double desiredRotations = m_desiredAngle / (2 * M_PI); // relative [-100,100]
        frc::SmartDashboard::PutNumber(m_name + " Desired Rotations:", desiredRotations);
        double closestSetpoint = calculateTargetSetpoint(rawRotations, desiredRotations);
        frc::SmartDashboard::PutNumber(m_name + " Closest Set Point", closestSetpoint);
        //Inversion Awareness
        if (abs(closestSetpoint - rawRotations) > 0.25){
            closestSetpoint = closestSetpoint + copysign(0.5, closestSetpoint - rawRotations);
            m_wheel.SetInverted(true);
        }
        else{
            m_wheel.SetInverted(false);
        }
        pid.SetReference(closestSetpoint * m_turnGearRatio, rev::ControlType::kPosition);
    }

    static double calculateTargetSetpoint(double current, double desired) {
        // Step 1: fmod both values
        double original = current;
        current = fmod(current, 1.0);
        desired = fmod(desired, 1.0);
        // Step 2: turn range [-1, 1] to [0, 1] (for negative numbers only. Unneeded?)
        // if (current < 0)
        //     current += 1.0;
        // if (desired < 0)
        //     desired += 1.0;
        // Step 3: Check going forward, going forward to the next rotation, going to the previous rotation
        // Example Case: If we are checking against a desired 0.9: It will check: -0.1, 0.9, and 1.9
        // Example Case: If we are checking against a desired -0.1: It will check: -1.1, -0.1, and 0.9
        double backwardSetpoint = desired - 1.0;
        double middleSetpoint = desired;
        double forwardSetpoint = desired + 1.0;
        // Step 4: Calculate the differences in setpoints
        double backwardDifference = backwardSetpoint - current;
        double middleDifference = middleSetpoint - current;
        double forwardDifference = forwardSetpoint - current;
        // Step 5: Compare the absolute difference
        double smallestDifference = middleDifference;
        if (abs(backwardDifference) < abs(smallestDifference)) {
            smallestDifference = backwardDifference;
        }
        if (abs(forwardDifference) < abs(smallestDifference)) {
            smallestDifference = forwardDifference;
        }
        // Step 6: Add the *actual* difference to the original value and return
        return original + smallestDifference; 
    }
    
};

class SwerveDrive {
    private:
    SwerveModule m_fl;
    SwerveModule m_fr;
    SwerveModule m_bl;
    SwerveModule m_br;
    AHRS *m_ahrs; // nav-x-MXP (gyro)
    double m_width;
    double m_length;
    double m_diameter;
    public:
    SwerveDrive(std::tuple<int, int, int> fl, std::tuple<int, int, int> fr, std::tuple<int, int, int> bl, std::tuple<int, int, int> br, double wid, double len) : 
        m_fl("front left", std::get<0>(fl), std::get<1>(fl), std::get<2>(fl)), 
        m_fr("front right", std::get<0>(fr), std::get<1>(fr), std::get<2>(fr)),
        m_bl("back left", std::get<0>(bl), std::get<1>(bl), std::get<2>(bl)),
        m_br("back right", std::get<0>(br), std::get<1>(br), std::get<2>(br)) {
        // Initialize values used for Feed
        // ahrs = AHRS()
        m_width = wid;
        m_length = len;
        m_diameter = sqrt((len * len) + (wid * wid)); // "R"
        m_ahrs = new AHRS(frc::SPI::Port::kMXP);
    }
    double GetGyroAngle() {
        return m_ahrs->GetAngle();
    }
    void Feed(double raw_fwd, double raw_strf, double rot) {
        // Translate the fwd and strf values into field-relative positions
        double theta = GetGyroAngle() * M_PI / 180.0;
        double fwdN = raw_fwd * cos(theta) + raw_strf * sin(theta);
        double strfN = raw_strf * cos(theta) - raw_fwd * sin(theta);
        
        // Calculate A,B,C,D for use in inverse kinematics calculations
        double A = (strfN - rot) * (m_length/m_diameter);
        double B = (strfN + rot) * (m_length/m_diameter);
        double C = (fwdN - rot) * (m_width/m_diameter);
        double D = (fwdN + rot) * (m_width/m_diameter); 
        
        // Calculate unscaled wheel speeds
        double wsFR = sqrt(pow(B,2)+pow(C,2));
        double wsFL = sqrt(pow(B,2)+pow(D,2));
        double wsBR = sqrt(pow(A,2)+pow(C,2));
        double wsBL = sqrt(pow(A,2)+pow(D,2));

        // Then scale all wheel speeds to range [-1, 1]
        double wsMax = 0;
        if (abs(wsFR) > wsMax) wsMax = abs(wsFR);
        if (abs(wsFL) > wsMax) wsMax = abs(wsFL);
        if (abs(wsBR) > wsMax) wsMax = abs(wsBR);
        if (abs(wsBL) > wsMax) wsMax = abs(wsBL);
    
        if (wsMax > 1.0) {
            wsFL = wsFL/wsMax;
            wsFR = wsFR/wsMax;
            wsBR = wsBR/wsMax;
            wsBL = wsBL/wsMax;
        }

        // Calculate Wheel Angles
        double waFR = atan2(B,C);
        double waFL = atan2(B,D);
        double waBR = atan2(A,C);
        double waBL = atan2(A,D);

        // Set member's desired angles & wheel speeds
        m_fl.setDesiredAngle(waFL);
        m_fl.setWheelSpeed(wsFL);

        m_fr.setDesiredAngle(waFR);
        m_fr.setWheelSpeed(wsFR);

        m_bl.setDesiredAngle(waBL);
        m_bl.setWheelSpeed(wsBL);
    
        m_br.setDesiredAngle(waBR);
        m_br.setWheelSpeed(wsBR);

        // Log the speeds and angles to SmartDashboard
        frc::SmartDashboard::PutNumber("FL Wheel Speed", wsFL);
        frc::SmartDashboard::PutNumber("FR Wheel Speed", wsFR);
        frc::SmartDashboard::PutNumber("BR Wheel Speed", wsBR);
        frc::SmartDashboard::PutNumber("BL Wheel Speed", wsBL);
        
        frc::SmartDashboard::PutNumber("FL Wheel Angle", waFL);
        frc::SmartDashboard::PutNumber("FR Wheel Angle", waFR);
        frc::SmartDashboard::PutNumber("BR Wheel Angle", waBR);
        frc::SmartDashboard::PutNumber("BL Wheel Angle", waBL);
    }
    void ApplyToSwerveModules() {
        m_fl.apply();
        m_fr.apply();
        m_bl.apply();
        m_br.apply();
    }
};

/*
TeleopPeriodic() {
    mySwerveDrive.Feed(controller.GetXRight(), controller.GetYRight(), controller.GetYLeft());
    mySwerveDrive.Apply();
}
*/
