// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Robot;

public class PoseSubsystem extends SubsystemBase {
    private static PoseSubsystem instance;
    private final Swerve s_Swerve;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2 gyro;

    public PoseSubsystem(Swerve s_Swerve) {
        assert (instance == null);
        instance = this;

        this.s_Swerve = s_Swerve;

        gyro = new Pigeon2(Pose.pigeonID, Constants.Swerve.swerveCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        Pose.rotationPID.enableContinuousInput(-180.0, 180.0);
        Pose.rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        Pose.rotationPID.reset();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                s_Swerve.getModulePositions(), new Pose2d());
    }

    public static PoseSubsystem getInstance() {
        return instance;
    }

    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
    }

    public void hack() {
        gyro.setYaw(gyro.getYaw().getValueAsDouble() + 180.0);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(), pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), s_Swerve.getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public static double angleErrorToSpeed(Rotation2d angleError) {
        return angleErrorToSpeed(angleError, Pose.rotationPID);
    }

    public static double angleErrorToSpeed(Rotation2d angleError, PIDController pid) {
        double angleErrorDeg = angleError.getDegrees();
        double correction = pid.calculate(angleErrorDeg);
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double output = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        DogLog.log("Pose/Angle Error", angleErrorDeg);
        DogLog.log("Pose/Angle PID correction", correction);
        DogLog.log("Pose/Angle feedforward", feedForward);
        DogLog.log("Pose/Angle output", output);

        // Invert due to use as joystick controls
        return -output;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), s_Swerve.getModulePositions());

        SmartDashboard.putNumber("pose/Gyro", getHeading().getDegrees());

        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
    }
}