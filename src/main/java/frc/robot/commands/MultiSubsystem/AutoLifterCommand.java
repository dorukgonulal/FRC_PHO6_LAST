package frc.robot.commands.MultiSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorControlCommand;
import frc.robot.commands.elevator.ElevatorHoldCommand;
import frc.robot.commands.pivot.PivotControlCommand;
import frc.robot.commands.pivot.PivotHoldCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;;

public class AutoLifterCommand  extends SequentialCommandGroup {

    private PivotSubsystem pivotSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    private double pivotHoldPower;

    private PivotPosition pivotLevel;
    private ElevatorPositions elevatorLevel;

    public AutoLifterCommand(PivotSubsystem pivotSubsystem, ElevatorSubsystem elevatorSubsystem,
        PivotPosition pivotLevel, ElevatorPositions elevatorLevel){

        this.pivotSubsystem = pivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        this.pivotLevel = pivotLevel;
        this.elevatorLevel = elevatorLevel;

        addRequirements(pivotSubsystem, elevatorSubsystem);

        addCommands( 
            new ParallelCommandGroup(
                new PivotControlCommand(pivotSubsystem, pivotLevel),
                new ElevatorControlCommand(elevatorSubsystem, elevatorLevel)),

                new ParallelCommandGroup(
                    new ElevatorHoldCommand(elevatorSubsystem),
                    new PivotHoldCommand(pivotSubsystem, 0.04)
                )
        );
    }
}