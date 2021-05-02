#pragma once
#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>
#include <math.h>
#include <units/length.h>
#include <units/velocity.h>

class SparkController: public frc::SpeedController {
public:
    static constexpr double maxRPM = 5500;
    static constexpr double wheelSizeInInches = 6;
    static constexpr double gearRatio = .1;
    static constexpr double pi = 3.1415;
    static constexpr double maxInchesPerSecond = maxRPM * gearRatio * (wheelSizeInInches * pi) / 60;
    static constexpr double maxMetersPerSecond = maxInchesPerSecond / 39.37; // Should be approx. 4.38
    static constexpr double metersToRpm = maxRPM/maxMetersPerSecond;
    bool encoderEnabled = false;
    rev::CANSparkMax motor;
    rev::CANEncoder encoder;
    rev::CANPIDController pid;
    SparkController(rev::CANSparkMax &motor, rev::CANPIDController &pid) 
    : motor(motor), encoder(motor.GetEncoder()), pid(pid) {
    }
    void ResetEncoder() { encoder.SetPosition(0); }
    double GetVelocity() { return encoder.GetVelocity(); }
    units::meter_t GetDistance() {
        double revs = GetPosition();
        return units::inch_t(revs * gearRatio * wheelSizeInInches * pi);
    }
    double GetPosition()  { return encoder.GetPosition(); }
    void UseEncoder(bool val) { encoderEnabled = val; }
    void Set(double speed) override {
        if (!encoderEnabled) {
            motor.Set(speed);
            return;
        }
        pid.SetReference(maxRPM * speed, rev::ControlType::kVelocity);
    }
    void SetSpeed(units::meters_per_second_t speed){
        pid.SetReference(metersToRpm * double(speed), rev::ControlType::kVelocity);
    }
    double Get() const override { return motor.Get(); }
    void SetInverted(bool isInverted) override { motor.SetInverted(isInverted); }
    bool GetInverted() const override { return motor.GetInverted(); }
    void Disable() override { motor.Disable(); }
    void StopMotor() override { motor.StopMotor(); }
    void PIDWrite(double speed) override { Set(speed); }
};