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
    frc::DifferentialDriveKinematics kDriveKinematics{kTrackwidth};
    AHRS &gyro;
    frc::DifferentialDriveOdometry odometry; // do not initialize this to 0, instead read from the gyro
    SparkController leftMotor;
    SparkController rightMotor;
    frc::Trajectory currentPath;
    frc::Timer timer;
    frc::RamseteController controller;
    std::string old_path = "NOTHING";
    PathProcessor(SparkController &leftMotor, SparkController &rightMotor, AHRS &aGyro) : leftMotor(leftMotor), rightMotor(rightMotor), gyro(aGyro), odometry(units::radian_t(units::degree_t(aGyro.GetYaw()))) {

    }
    void setPath(std::string newPath) {
        frc::Trajectory rawTraj = loadTrajectory(newPath);
        frc::Transform2d transform = odometry.GetPose() - rawTraj.InitialPose();
	    currentPath = rawTraj.TransformBy(transform);
    }
    frc::Trajectory loadTrajectory(std::string fileName) {
        wpi::SmallString<64> deployDirectory;
        frc::filesystem::GetDeployDirectory(deployDirectory);
        wpi::sys::path::append(deployDirectory, "output");
        wpi::sys::path::append(deployDirectory, fileName + ".wpilib.json");

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
            std::cout << timer.Get() << " " << currentPath.TotalTime() << path << "\n";
            // This is bad, should not be running after path is finished!
        }
        runCurrentPath(reverse);

    }
    std::string getCurrentPath() {
        return old_path;
    }
    units::radian_t getCurrentAngle() {
        return units::degree_t(gyro.GetYaw());
    }
    void runCurrentPath(bool reverse) {
        //units::meters_per_second_t left;
        //units::meters_per_second_t right;
        frc::Pose2d currentRobotPose = odometry.Update(getCurrentAngle(), leftMotor.GetDistance(), rightMotor.GetDistance());
        const frc::Trajectory::State goal = currentPath.Sample(units::second_t(timer.Get())); 
        frc::ChassisSpeeds adjustedSpeeds = controller.Calculate(currentRobotPose, goal);
        auto [left, right] = kDriveKinematics.ToWheelSpeeds(adjustedSpeeds);
        // driving backwards should need l and r reversed along with reversed input to motors

        // .75, .25 => up right
        // then in reverse we want to go back left
        // -.75, -.25 => back left
        if(reverse) {
            left = -left; // TODO verify this!!!
            right = -right;
        }
        leftMotor.SetSpeed(units::meters_per_second_t(-left * 0.5)); // TODO test w/o .5x speed
        rightMotor.SetSpeed(units::meters_per_second_t(right * 0.5));
        // Make sure to feed
    }
    bool pathCompleted() {
        return units::second_t(timer.Get()) > currentPath.TotalTime();
    }

};