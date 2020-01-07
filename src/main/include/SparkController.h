#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>

class SparkController: public frc::SpeedController {
    double maxRPM = 0;
    bool encoderEnabled = false;
    SparkController(rev::CANSparkMax &aMotor, rev::CANPIDController &pid) : motor(aMotor), encoder(aMotor.GetEncoder()) {

    }
    public : double GetMaxRPM() { return pid.GetSmartMotionMaxVelocity(); }
    public : double GetEncoder() { return encoder.GetPosition(); }
    public : void UseEncoder(bool val) { encoderEnabled = val; }
    void Set(double speed) {
        if (!encoderEnabled) {
            motor.Set(speed);
            return;
        }
        motor.SetReference(GetMaxRPM() * speed, rev::ControlType::kSmartVelocity);
    }
    double Get() { return motor.Get(); }
    public : double GetApplied() { return motor.GetAppliedOutput() }
    void SetInverted(bool isInverted) { motor.SetInverted(isInverted); }
    bool GetInverted() { return motor.GetInverted(); }
    void Disable() { motor.Disable(); }
    void StopMotor() { motor.StopMotor(); }
};