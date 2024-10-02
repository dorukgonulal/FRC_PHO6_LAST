package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotHoldCommand extends Command {
 
  private final PivotSubsystem pivotSubsystem;
  private double speed;

  public PivotHoldCommand(PivotSubsystem pivotSubsystem, double speed) {
    this.speed = speed;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    pivotSubsystem.pivotUp(speed);
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
