#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>
class SparkController: public frc::SpeedController {
    double maxRPM = 0;
    bool encoderEnabled = false;
    SparkController(rev::CANSparkMax actualMotor, rev::CANPIDController pid) : actualMotor(motor), encoder(motor.GetEncoder()) {
        encoder = actualMotor.GetEncoder();
        motor = actualMotor;
    }
    void SetMaxRPM(double rpm) { maxRPM = rpm; }
    double GetMaxRPM() { return maxRPM; }
    void GetEncoder() { return motor.GetEncoder(); }
    void UseEncoder(bool val) { encoderEnabled = val; }
    void Set(double speed) {
        if (!encoderEnabled) {
            motor.Set(speed);
            return;
        }
        motor.SetReference(maxRPM * speed, rev::ControlType::kSmartVelocity);
    }
    double Get() { return motor.Get(); }
    void SetInverted(bool isInverted) { motor.SetInverted(isInverted); }
    bool GetInverted() { return motor.GetInverted(); }
    void Disable() { motor.Disable(); }
    void StopMotor() { motor.StopMotor(); }
}