#include <cmath>
#include <string>

// #include <SPI.h>

#include <rev/CANSparkMax.h>
#include <AHRS.h>

class SwerveModule {
    private:
    std::string m_name;
    rev::CANSparkMax m_wheel; // wheel motor
    rev::CANSparkMax m_turn; // turn motor
    double wheelSpeed;
    double desiredAngle;
    public:  
    SwerveModule(std::string name, int wID, int tID) : m_wheel(wID, rev::CANSparkMax::MotorType::kBrushless), m_turn(tID, rev::CANSparkMax::MotorType::kBrushless) {
        m_name = name;
    }
    void setDesiredAngle(double angle) {
        desiredAngle = angle;
    }
    void setWheelSpeed(double ws) {
        wheelSpeed = ws;
    }
    void apply() {
        m_wheel.Set(wheelSpeed);
        // Calculate desired turn motor power based on current encoder position and desired angle
        // double calculated_turn_speed = calculation();
        double calculated_turn_speed = 1.0;
        m_turn.Set(calculated_turn_speed);
        /*
        if (encoderAngle < desiredAngle) {
            calculated_turn_speed = -1;
        } else {
            calculated_turn_speed = 1;
        } // but we want a PID loop, so that the closer it gets the smaller the turn speed is

        // Additionally, if we are more than 90 degrees away, it's faster to invert TODO later
        */
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
        m_br("back right", br.first, br.second){
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
    void Apply() {
        // TODO
    }
};

/*
TeleopPeriodic() {
    mySwerveDrive.Feed(controller.GetXRight(), controller.GetYRight(), controller.GetYLeft());
    mySwerveDrive.Apply();
}
*/