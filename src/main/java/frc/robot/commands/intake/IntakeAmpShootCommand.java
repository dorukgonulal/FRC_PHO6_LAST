package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAmpShootCommand extends Command {

  private final IntakeSubsystem mIntakeSubsystem;
  private final FeederSubsystem feederSubsystem;

  public IntakeAmpShootCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {

    this.mIntakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSubsystem.intakeShoot(Constants.IntakeConstants.AMP_SHOOT_POWER);
    feederSubsystem.feedShoot(Constants.IntakeConstants.INTAKE_FEED_POWER);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.stopIntake();
    feederSubsystem.feedStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
