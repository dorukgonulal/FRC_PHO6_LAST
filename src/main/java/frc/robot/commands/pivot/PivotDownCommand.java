package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotDownCommand extends Command {
 
  private final PivotSubsystem pivotSubsystem;

  public PivotDownCommand(PivotSubsystem pivotSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    pivotSubsystem.pivotDown(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.pivotStop();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
