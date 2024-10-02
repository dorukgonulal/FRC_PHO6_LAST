package frc.robot.commands.intake;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOnCommand extends Command {

  private final IntakeSubsystem mIntakeSubsystem;
  private final FeederSubsystem feederSubsystem;

  public IntakeOnCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {

    this.mIntakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(mIntakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    mIntakeSubsystem.setIdleMod(IdleMode.kBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSubsystem.intakeOn(Constants.IntakeConstants.INTAKE_ON_POWER);
    feederSubsystem.feedOn(Constants.IntakeConstants.INTAKE_FEED_POWER);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.stopIntake();
    feederSubsystem.feedStop();
    mIntakeSubsystem.setIdleMod(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
