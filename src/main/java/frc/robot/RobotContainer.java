package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.MultiSubsystem.AutoLifterCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.intake.IntakeAmpShootCommand;
import frc.robot.commands.intake.IntakeFeedShootCommand;
import frc.robot.commands.intake.IntakeOnCommand;
import frc.robot.commands.intake.IntakeRollerSpeakerShootCommand;
import frc.robot.commands.intake.IntakeTrapShootCommand;
import frc.robot.commands.pivot.PivotDownCommand;
import frc.robot.commands.pivot.PivotUpCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.IOConstants.k_DRIVER_CONTROLLER_PORT);
    private final Joystick operator = new Joystick(Constants.IOConstants.k_OPERATOR_CONTROLLER_PORT);
    // private final Joystick soloStick = new Joystick(2);

    private final CommandPS5Controller driverController = new CommandPS5Controller(
      Constants.IOConstants.k_DRIVER_CONTROLLER_PORT);
    
    public final CommandPS5Controller operatorController = new CommandPS5Controller(
      Constants.IOConstants.k_OPERATOR_CONTROLLER_PORT);

    /* Drive Controls */
    private final int translationAxis = 0;
    private final int strafeAxis = 1;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 1);
    private final JoystickButton robotCentric = new JoystickButton(driver, 12);
    private final JoystickButton slowButton = new JoystickButton(driver, 7); // Yeni eklenen boost butonu

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    
    private final JoystickButton intakeon = new JoystickButton(operator, 4);
    private final JoystickButton intakeshootroll = new JoystickButton(operator, 3);
    private final JoystickButton intakeshootfeed = new JoystickButton(operator, 5);
    private final JoystickButton amprelease = new JoystickButton(operator, 6);

    private final JoystickButton trapShoot = new JoystickButton(operator, 1);
    private final JoystickButton elevatorDown = new JoystickButton(operator, 2);
    private final JoystickButton elevatorUp = new JoystickButton(operator, 8);

    // private final JoystickButton pivotUp = new JoystickButton(operator, 11);
    // private final JoystickButton pivotDown = new JoystickButton(operator, 12);
    
    private final POVButton pivotzero = new POVButton(operator, 0);
    private final POVButton pivotamp = new POVButton(operator, 90);
    private final POVButton pivotintake = new POVButton(operator, 180);


    /* Subsystems */ 
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverController.getRawAxis(translationAxis), 
                () -> driverController.getRawAxis(strafeAxis), 
                () -> -driverController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> slowButton.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.setHeading(new Rotation2d(0))));
        intakeon.whileTrue(new IntakeOnCommand(intake, feeder));
        intakeshootroll.whileTrue(new IntakeRollerSpeakerShootCommand(intake));
        intakeshootfeed.whileTrue(new IntakeFeedShootCommand(feeder));
        amprelease.whileTrue(new IntakeAmpShootCommand(intake, feeder));

        trapShoot.whileTrue(new IntakeTrapShootCommand(intake));
        elevatorDown.whileTrue(new ElevatorDownCommand(elevator));
        elevatorUp.whileTrue(new ElevatorUpCommand(elevator));
        // pivotUp.whileTrue(new PivotUpCommand(pivot));
        // pivotDown.whileTrue(new PivotDownCommand(pivot));
        //pivotPidIntake.whileTrue(new PivotControlCommand(pivot, PivotPosition.INTAKE));

        pivotzero.onTrue(new AutoLifterCommand(pivot, elevator, PivotPosition.CLOSE, ElevatorPositions.BASE));
        pivotamp.onTrue(new AutoLifterCommand(pivot, elevator, PivotPosition.SHOOTAMP, ElevatorPositions.SHOOTAMP));
        pivotintake.onTrue(new AutoLifterCommand(pivot, elevator, PivotPosition.INTAKE, ElevatorPositions.BASE));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
