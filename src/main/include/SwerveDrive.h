class Wheel {
    string name;
    rev::CANSparkMax wheel;
    rev::CANSparkMax turn;
    Wheel(string name, int wID, int tID);
};

class SwerveDrive {
    Wheel m_fl;
    Wheel m_fr;
    Wheel m_bl;
    Wheel m_br;
    pair<double, double> fl_out;
    pair<double, double> fr_out;
    pair<double, double> bl_out;
    pair<double, double> br_out;
    public:
    SwerveDrive(fl Wheel, fr Wheel, bl Wheel, br Wheel);
    void Feed(double fwd, double strf, double rot); // Update each pair with the correct output values
    void Apply(); // Set the wheel based on the output values
};