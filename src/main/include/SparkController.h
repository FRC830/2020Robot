#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>

class SparkController: public frc::SpeedController {
public:
    double maxRPM = 0;
    bool encoderEnabled = true;
    rev::CANSparkMax motor;
    rev::CANEncoder encoder;
    rev::CANPIDController pid;
    SparkController(rev::CANSparkMax &motor, rev::CANPIDController &pid) 
    : motor(motor), encoder(motor.GetEncoder()), pid(pid) {

    }
    double GetMaxRPM() { return pid.GetSmartMotionMaxVelocity(); }
    double GetEncoder()  { return encoder.GetPosition(); }
    void UseEncoder(bool val) { encoderEnabled = val; }
    void Set(double speed) override {
        if (!encoderEnabled) {
            motor.Set(speed);
            return;
        }
        pid.SetReference(GetMaxRPM() * speed, rev::ControlType::kSmartVelocity);
    }
    double Get() const override { return motor.Get(); }
    double GetApplied() { return motor.GetAppliedOutput(); }
    void SetInverted(bool isInverted) override { motor.SetInverted(isInverted); }
    bool GetInverted() const override { return motor.GetInverted(); }
    void Disable() override { motor.Disable(); }
    void StopMotor() override { motor.StopMotor(); }
    void PIDWrite(double speed) override { Set(speed); }
};