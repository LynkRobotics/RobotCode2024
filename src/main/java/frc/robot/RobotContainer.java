package frc.robot;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Options.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    /* Driver Buttons */
    private final Trigger intakeButton = driver.leftBumper();
    private final Trigger shooterButton = driver.rightBumper();
    private final Trigger ejectButton = driver.start();
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IndexSubsystem s_Index = new IndexSubsystem();
    @SuppressWarnings ("unused")
    private final LEDSubsystem s_Led = new LEDSubsystem();
    @SuppressWarnings ("unused")
    private final PoseSubsystem s_Pose = new PoseSubsystem(s_Swerve);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -translation.get() * Constants.driveStickSensitivity,
                        () -> -strafe.get() * Constants.driveStickSensitivity,
                        () -> -rotation.get() * Constants.turnStickSensitivity,
                        s_Swerve::getSpeedLimitRot));

        s_Shooter.setDefaultCommand(Commands.startEnd(s_Shooter::stop, () -> {}, s_Shooter).withName("Shooter stop"));
        s_Index.setDefaultCommand(Commands.startEnd(s_Index::stop, () -> {}, s_Index).withName("Index Stop"));

        // During calibration allow for direct control
        //SmartDashboard.putNumber("Shooter voltage direct", 0.0);
        //SmartDashboard.putData("Set shooter voltage", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(SmartDashboard.getNumber("Shooter voltage direct", 0)); }));
        //SmartDashboard.putData("Stop shooter", s_Shooter.runOnce(() -> { s_Shooter.setVoltage(0); }));

        SmartDashboard.putNumber("TeleOp Speed Governor", 0.35);

        // Allow for direct RPM setting
        SmartDashboard.putNumber("Shooter top RPM", 1000.0);
        SmartDashboard.putNumber("Shooter bottom RPM", 1000.0);
        SmartDashboard.putData("Idle shooter", s_Shooter.runOnce(() -> { s_Shooter.setRPM(500); }));
        SmartDashboard.putData("Zero Gyro", Commands.print("Zeroing gyro").andThen(Commands.runOnce(s_Pose::zeroGyro, s_Swerve)).andThen(Commands.print("Gyro zeroed")).withName("Zero Gyro")); //TODO: Test
        SmartDashboard.putData("Zero heading", Commands.print("Zeroing heading").andThen(Commands.runOnce(s_Pose::zeroHeading, s_Swerve)).andThen(Commands.print("Heading zeroed")).withName("Zero heading")); //TODO: Test
        SmartDashboard.putData("Reset heading", Commands.print("Resetting heading").andThen(Commands.runOnce(s_Pose::resetHeading, s_Swerve)).andThen(Commands.print("Heading reset")).withName("Reset heading"));

        SmartDashboard.putData("autoSetup/SetSwerveCoast", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Coasting Swerve Motors");}).andThen(Commands.runOnce(s_Swerve::setMotorsToCoast, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Coasted");})).ignoringDisable(true));
        SmartDashboard.putData("autoSetup/SetSwerveBrake", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Braking Swerve Motors");}).andThen(Commands.runOnce(s_Swerve::setMotorsToBrake, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Braked");})).ignoringDisable(true));
        SmartDashboard.putData("autoSetup/SetSwerveAligned", Commands.runOnce(() -> { DogLog.log("Auto/Status", "Aligning Swerve Motors");}).andThen(Commands.run(s_Swerve::alignStraight, s_Swerve)).andThen(Commands.runOnce(() -> { DogLog.log("Auto/Status", "Swerve Motors Aligned");})).ignoringDisable(true));

        DogLog.setOptions(
            new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogEntryQueueCapacity(1000)
            .withLogExtras(true)
            .withNtPublish(Constants.atHQ));
    
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        intakeButton.whileTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::enableSpeedLimit),
                Commands.either(
                    new ShooterIntakeCommand(s_Shooter, s_Index, driver.getHID()),
                    new IntakeCommand(s_Intake, s_Index, driver.getHID()),
                    optShooterIntake),
                Commands.runOnce(s_Swerve::disableSpeedLimit))
            .handleInterrupt(s_Swerve::disableSpeedLimit)
            .withName("Intake"));
        shooterButton.whileTrue(
            Commands.either(new ShootCommand(s_Shooter, s_Index,
                () -> SmartDashboard.getNumber("Shooter top RPM", 0.0),
                () -> SmartDashboard.getNumber("Shooter bottom RPM", 0.0)),
            new ShootCommand(s_Shooter, s_Index),
            optDirectRPM)
            .withName("Shoot"));
        SmartDashboard.putData("Disable speed limit", Commands.runOnce(s_Swerve::disableSpeedLimit));

        ejectButton.whileTrue(new EjectCommand(s_Intake, s_Index, s_Shooter));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public void teleopInit() {
    }

    public void teleopExit() {
    }
}