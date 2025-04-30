// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private static TalonFX top;
    private static TalonFX bottom;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);
    private double topCurrentTarget = 0.0;
    private double bottomCurrentTarget = 0.0;
    SendableChooser<Speed> defaultShotChooser = new SendableChooser<>();
    private boolean autoAimingActive = false;

    private class ShooterSpeed {
        double topMotorSpeed;
        double bottomMotorSpeed;

        public ShooterSpeed(double top, double bottom) {
            topMotorSpeed = top;
            bottomMotorSpeed = bottom;
        }
    }

    public enum Speed {
        STOP,
        INTAKE,
        AMP,
        SUBWOOFER,
        AMPSIDE,
        MIDLINE,
        PODIUM,
        FULL,
        SHORTSLIDE,
        SLIDE,
        SPECIAL,
        EJECT,
        BLOOP
    };

    private Speed nextShot = null;

    private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
            Map.entry(Speed.STOP, new ShooterSpeed(Constants.Shooter.stopSpeed, Constants.Shooter.stopSpeed)),
            Map.entry(Speed.INTAKE, new ShooterSpeed(Constants.Shooter.intakeSpeed, Constants.Shooter.intakeSpeed)),
            Map.entry(Speed.AMP, new ShooterSpeed(375, 1025)),
            Map.entry(Speed.SUBWOOFER, new ShooterSpeed(1360, 2830)),
            Map.entry(Speed.AMPSIDE, new ShooterSpeed(2850, 2050)),
            Map.entry(Speed.MIDLINE, new ShooterSpeed(2800, 2300)),
            Map.entry(Speed.PODIUM, new ShooterSpeed(3000, 1600)),
            Map.entry(Speed.FULL, new ShooterSpeed(Constants.Shooter.topSpeed, Constants.Shooter.topSpeed)),
            Map.entry(Speed.SHORTSLIDE, new ShooterSpeed(2250, 900)),
            Map.entry(Speed.SLIDE, new ShooterSpeed(2500, 1000)),
            Map.entry(Speed.SPECIAL, new ShooterSpeed(1180, 1180)),
            Map.entry(Speed.EJECT, new ShooterSpeed(-800, -800)),
            Map.entry(Speed.BLOOP, new ShooterSpeed(400, 400))));

    public ShooterSubsystem() {
        top = new TalonFX(Constants.Shooter.topShooterID, Constants.Shooter.shooterMotorCanBus);
        bottom = new TalonFX(Constants.Shooter.bottomShooterID, Constants.Shooter.shooterMotorCanBus);
        applyConfigs();

        SmartDashboard.putNumber("shooter/Top RPM adjustment", 0.0);
        SmartDashboard.putNumber("shooter/Bottom RPM adjustment", 0.0);

        defaultShotChooser.setDefaultOption("SUBWOOFER", Speed.SUBWOOFER);
        for (Speed speed : Speed.values()) {
            if (speed != Speed.SUBWOOFER) {
                defaultShotChooser.addOption(speed.toString(), speed);
            }
        }
        SmartDashboard.putData("shooter/Default shot", defaultShotChooser);
    }

    private void applyConfigs() {
        /* Configure the Shooter Motors */
        var m_ShooterMotorsConfiguration = new TalonFXConfiguration();
        /* Set Shooter motors to Brake */
        m_ShooterMotorsConfiguration.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
        /* Set the Shooters motor direction */
        m_ShooterMotorsConfiguration.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
        /* Config the peak outputs */
        m_ShooterMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
        m_ShooterMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

        // PID & FF configuration
        m_ShooterMotorsConfiguration.Slot0.kP = Constants.Shooter.kP;
        m_ShooterMotorsConfiguration.Slot0.kI = Constants.Shooter.kI;
        m_ShooterMotorsConfiguration.Slot0.kD = Constants.Shooter.kD;
        m_ShooterMotorsConfiguration.Slot0.kS = Constants.Shooter.kS;
        m_ShooterMotorsConfiguration.Slot0.kV = 1.0 / toRPS(Constants.Shooter.RPMsPerVolt);
        m_ShooterMotorsConfiguration.Slot0.kA = 0.0;
        m_ShooterMotorsConfiguration.Slot0.kG = 0.0;

        /* Apply Shooters Motor Configs */
        top.getConfigurator().apply(m_ShooterMotorsConfiguration);
        bottom.getConfigurator().apply(m_ShooterMotorsConfiguration);
    }

    private double toRPM(double rps) {
        return rps * 60.0;
    }

    private double toRPS(double rpm) {
        return rpm / 60.0;
    }

    public void setNextShot(Speed speed) {
        nextShot = speed;
    }

    public boolean isAutoAimingActive() {
        return autoAimingActive;
    }

    public boolean shoot() {
        return setCurrentSpeed(nextShot);
    }

    private Speed defaultSpeed() {
        return defaultShotChooser.getSelected();
    }

    private boolean setCurrentSpeed(Speed speed) {
        ShooterSpeed shooterSpeed;

        if (speed == null) {
            speed = defaultSpeed();
        }

        shooterSpeed = shooterSpeeds.get(speed);
        setCurrentSpeed(shooterSpeed);

        return true;
    }

    public void shoot(double topRPM, double bottomRPM) {
        setCurrentSpeed(new ShooterSpeed(topRPM, bottomRPM));
    }

    private void setCurrentSpeed(ShooterSpeed speed) {
        topCurrentTarget = speed.topMotorSpeed + SmartDashboard.getNumber("shooter/Top RPM adjustment", 0.0);
        bottomCurrentTarget = speed.bottomMotorSpeed + SmartDashboard.getNumber("shooter/Bottom RPM adjustment", 0.0);
        top.setControl(topControl.withVelocity(toRPS(topCurrentTarget)));
        bottom.setControl(bottomControl.withVelocity(toRPS(bottomCurrentTarget)));
        DogLog.log("Shooter/TopRPM", topCurrentTarget);
        DogLog.log("Shooter/BottomRPM", bottomCurrentTarget);
    }

    public void setVoltage(double voltage) {
        top.setControl(voltageOut.withOutput(voltage));
        bottom.setControl(voltageOut.withOutput(voltage));
    }

    public void setRPM(double rpm) {
        setCurrentSpeed(new ShooterSpeed(rpm, rpm));
    }

    public void intake() {
        setCurrentSpeed(Speed.INTAKE);
    }

    public void eject() {
        setCurrentSpeed(Speed.EJECT);
    }

    public void stop() {
        setCurrentSpeed(Speed.STOP);
    }

    public boolean isReady(boolean precise) {
        return (Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - topCurrentTarget) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError) &&
                Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - bottomCurrentTarget) < (precise
                        ? Constants.Shooter.maxRPMErrorLong
                        : Constants.Shooter.maxRPMError));
    }

    public boolean shootingDefault() {
        return nextShot == null;
    }

    public void toggleAmp() {
        if (nextShot == null || nextShot != Speed.AMP) {
            nextShot = Speed.AMP;
        } else {
            nextShot = null;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double topVel = toRPM(top.getVelocity().getValueAsDouble());
        double bottomVel = toRPM(bottom.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("shooter/ready", isReady(false));
        SmartDashboard.putString("shooter/Next shot",
            nextShot == null ? defaultSpeed().toString() : nextShot.toString());

        DogLog.log("Shooter/Top RPM", topVel);
        DogLog.log("Shooter/Bottom RPM", bottomVel);
        DogLog.log("Shooter/Top RPM tgt", topCurrentTarget);
        DogLog.log("Shooter/Bottom RPM tgt", bottomCurrentTarget);
        DogLog.log("Shooter/Top RPM err", topVel - topCurrentTarget);
        DogLog.log("Shooter/Bottom RPM err", bottomVel - bottomCurrentTarget);
        DogLog.log("Shooter/Ready", isReady(false));
        DogLog.log("Shooter/Next shot", nextShot == null ? defaultSpeed().toString() : nextShot.toString());
    }
}