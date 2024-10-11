package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.PivotPosition;;


public class PivotControlCommand extends Command {

  private PivotSubsystem pivotSubsystem;
  private PivotPosition position;
  private double goalPos;
  private double error;

  private double PIVOT_POWER = Constants.PivotConstants.PIVOT_POWER;

  public PivotControlCommand(PivotSubsystem pivotSubsystem, PivotPosition position) {

    this.position = position;
    this.pivotSubsystem = pivotSubsystem;
    
    addRequirements(pivotSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (position) {
        
      case CLOSE:
      goalPos = Constants.PivotConstants.INTAKE_CLOSE_SETPOINT;
      break;

      case INTAKE:
      goalPos = Constants.PivotConstants.INTAKE_ON_SETPOINT;
      break;

      case SHOOTAMP:
      goalPos = Constants.PivotConstants.AMP_SHOOT_SETPOINT;
      break;

      case SHOOTSPEAKER:
      goalPos = Constants.PivotConstants.SPEAKER_SHOOT_SETPOINT;
      break;

      case TRAP:
      goalPos = Constants.PivotConstants.TRAP_SHOOT_SETPOINT;
      break;

      case PODIUMNOTE:
      goalPos = Constants.PivotConstants.PODIUMNOTE_SETPOINT;
      break;

    }

  }

  // Called every time the scheduler runs while the command is schedu led. -26
  @Override
  public void execute() {
 
      error = (goalPos - pivotSubsystem.getEncoderPosition());

      if (pivotSubsystem.getEncoderPosition() > -26) {
        if (Math.abs(error) > Constants.PivotConstants.PIVOT_KP) {

          double power = Constants.PivotConstants.PIVOT_KP * error;
  
          if (Math.abs(power) > PIVOT_POWER) {
  
            power = Math.copySign(PIVOT_POWER, power);
  
          }
  
          if (Math.abs(power) < 0.1) {
  
            power = Math.copySign(0.1, power);
  
          } else {
  
          pivotSubsystem.setPivot(power);
         } 
  
        } else {
  
          pivotSubsystem.pivotStop();
  
        }
      } else {
        pivotSubsystem.pivotStop();
      }

      SmartDashboard.putNumber("Pivot Error", error);

    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    pivotSubsystem.pivotStop(); 

  }

  public boolean isFinished() {
    
    return -0.2 < error && error < 0.2;

  }
}
