// AHRS stubs needed for tests to compile natively on non-Windows
#ifndef _WIN32

#include <AHRS.h>

// inherited from Gyro
void AHRS::Reset() {}
double AHRS::GetAngle() const { return 0; }
double AHRS::GetRate() const { return 0; }
void AHRS::Calibrate() {}

// inherited from PIDSource
double AHRS::PIDGet() { return 0; }

// inherited from SendableBase
void AHRS::InitSendable(frc::SendableBuilder& builder) {}

#endif
