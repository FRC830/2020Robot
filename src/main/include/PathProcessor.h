#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/Trajectory.h>
//#include <frc2/command/SubsystemBase.h>
#include <units/units.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/Timer.h>
#include "SparkController.h"
#include <AHRS.h>
#include <wpi/SmallString.h>
#include <wpi/Path.h>
#include <fstream>
#include <frc/Filesystem.h>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>

class PathProcessor {
    public:
    static constexpr units::inch_t kTrackwidth = 27.9_in;
    frc::DifferentialDriveKinematics kDriveKinematics{units::meter_t(kTrackwidth)};
    frc::DifferentialDriveOdometry odometry{units::radian_t(0)};
    SparkController leftMotor;
    SparkController rightMotor;
    frc::Trajectory currentPath;
    frc::Timer timer;
    frc::RamseteController controller;
    bool pathChanged = true;
    std::string old_path = "NOTHING";
    AHRS gyro{frc::SPI::Port::kMXP};
    PathProcessor(SparkController &leftMotor, SparkController &rightMotor) : leftMotor(leftMotor), rightMotor(rightMotor) {

    }
    void setPath(std::string newPath) {
        frc::Trajectory rawTraj = loadTrajectory(newPath);
        frc::Transform2d transform = odometry.GetPose() - rawTraj.InitialPose();
	    currentPath = rawTraj.TransformBy(transform);
        pathChanged = true;
    }
    frc::Trajectory loadTrajectory(std::string fileName) {
        wpi::SmallString<64> deployDirectory;
        frc::filesystem::GetDeployDirectory(deployDirectory);
        wpi::sys::path::append(deployDirectory, "output");
        wpi::sys::path::append(deployDirectory, fileName + ".path");

        return frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    void runPathUntilFinished(std::string path, bool reverse) {
        if (path != old_path) {
            setPath(path);
            timer.Reset();
            timer.Start();
            old_path = path;
        }
        if (pathCompleted()) {
            // This is bad, should not be running after path is finished!
        }
        runCurrentPath(reverse);

    }
    void runCurrentPath(bool reverse) {
        units::degree_t currentGyroAngle;
        units::meters_per_second_t left;
        units::meters_per_second_t right;
        double rawGyro = gyro.GetPitch();
        if(reverse) {
            currentGyroAngle = units::degree_t(-rawGyro+180); // negated so that is clockwise negative
        }else{
            currentGyroAngle = units::degree_t(-rawGyro); // negated so that is clockwise negative
        }

        frc::Pose2d currentRobotPose = odometry.Update(units::radian_t(currentGyroAngle), leftMotor.GetDistance(), rightMotor.GetDistance());
        const frc::Trajectory::State goal = currentPath.Sample(units::second_t(timer.Get())); 
        frc::ChassisSpeeds adjustedSpeeds = controller.Calculate(currentRobotPose, goal);
        frc::DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.ToWheelSpeeds(adjustedSpeeds);
        // driving backwards should need l and r reversed along with reversed input to motors
        if(reverse){
            left = -wheelSpeeds.right;
            right = -wheelSpeeds.left;
        } else {
            left = wheelSpeeds.left;
            right = wheelSpeeds.right;
        }
        leftMotor.SetSpeed(0.5*-left);
        rightMotor.SetSpeed(0.5*right);
        // Make sure to feed
    }
    bool pathCompleted() {
        return units::second_t(timer.Get()) > currentPath.TotalTime();
    }

};