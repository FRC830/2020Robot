#define _USE_MATH_DEFINES
#include <math.h>
// #include <cmath>
#include <string>

// #include <SPI.h>

#include <rev/CANSparkMax.h>
#include <AHRS.h>

class SwerveModule {
    private:
    std::string m_name;
    rev::CANSparkMax m_wheel; // wheel motor
    rev::CANSparkMax m_turn; // turn motor
    double m_wheelSpeed;
    double m_desiredAngle;
    const double m_p = 0.01;
    const double m_i = 0.0;
    const double m_d = 0.0;
    const double m_gearRatio = 100.0; // To do a 360, we have to turn the gear 100 times
    void setPID(double p, double i, double d) {
        auto pid = m_turn.GetPIDController();
        pid.SetP(p);
        pid.SetI(i);
        pid.SetD(d);
    }
    public:  
    SwerveModule(std::string name, int wID, int tID) : m_wheel(wID, rev::CANSparkMax::MotorType::kBrushless), m_turn(tID, rev::CANSparkMax::MotorType::kBrushless) {
        m_name = name;
        setPID(m_p, m_i, m_d);
    }
    void setDesiredAngle(double angle) {
        m_desiredAngle = angle;
    }
    void setWheelSpeed(double ws) {
        m_wheelSpeed = ws;
    }

    void apply() {
        m_wheel.Set(m_wheelSpeed);

        // Currently have an angle in radians
        // Convert this into "Ticks"
        // [-π, π] => [-1, 1] # [-100, 100], then i*(100/2π) is the current position in ticks
        auto pid = m_turn.GetPIDController();

        double rawRotations = (m_wheel.GetEncoder().GetPosition()); // 5.75
        int completeRotations = floor(rawRotations); // 5

        auto desiredRotations = (m_gearRatio / (2 * M_PI)) * m_desiredAngle; // relative [-100,100]

        double goForwardSetpoint = completeRotations + desiredRotations;
        double goBackwardSetpoint = goForwardSetpoint - 1.0;
        // Determine which of the two setpoints are closer

        double closestSetpoint = 0.0;
        if (abs(goBackwardSetpoint - rawRotations) < abs(goForwardSetpoint - rawRotations)) {
            closestSetpoint = goBackwardSetpoint;
        } else {
            closestSetpoint = goForwardSetpoint;
        }
        // TODO is this the right units??
        pid.SetReference(closestSetpoint, rev::ControlType::kPosition);
        // Additionally, if we are more than 90 degrees away, it's faster to invert TODO later
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
    SwerveDrive(std::pair<int, int> fl, std::pair<int, int> fr, std::pair<int, int> bl, std::pair<int, int> br, double wid, double len) : 
        m_fl("front left", fl.first, fl.second),
        m_fr("front right", fr.first, fr.second),
        m_bl("back left", bl.first, bl.second),
        m_br("back right", br.first, br.second) {
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
        double waBL = atan2(D,D);

        // Set member's desired angles & wheel speeds
        m_fl.setDesiredAngle(waFL);
        m_fl.setWheelSpeed(wsFL);

        m_fr.setDesiredAngle(waFR);
        m_fr.setWheelSpeed(wsFR);

        m_bl.setDesiredAngle(waBL);
        m_bl.setWheelSpeed(wsBL);
    
        m_br.setDesiredAngle(waBR);
        m_br.setWheelSpeed(wsBR);
    }
    void ApplyToSwerveModules() {
        // TODO
    }
};

/*
TeleopPeriodic() {
    mySwerveDrive.Feed(controller.GetXRight(), controller.GetYRight(), controller.GetYLeft());
    mySwerveDrive.Apply();
}
*/