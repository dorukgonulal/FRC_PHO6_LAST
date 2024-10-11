package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ElevatorControlCommand extends Command{

  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorPositions level;
  private double goalPos;
  private double error;

  public ElevatorControlCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPositions level) {

    this.level = level;
    this.elevatorSubsystem = elevatorSubsystem;
    
    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (level) {
        
      case BASE:
      goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_INTAKE;
      break;

      case MIDDLE:
      goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_MIDDLE;
      break;

      case SHOOTAMP:
      goalPos = Constants.ElevatorConstants.ELEVATOR_SETPOINT_SHOOTAMP;
      break;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
      error = goalPos - elevatorSubsystem.getEncoderPosition();
      
      if (elevatorSubsystem.getEncoderPosition() < 60){
        if (Math.abs(error) > Constants.ElevatorConstants.ELEVATOR_TOLERANCE) {

          double power = Constants.ElevatorConstants.ELEVATOR_KP * error;
  
          if (Math.abs(power) > Constants.ElevatorConstants.ELEVATOR_POWER) {
  
            power = Math.copySign(Constants.ElevatorConstants.ELEVATOR_POWER, power);
  
          }
          if (Math.abs(power) < 0.1) {
  
            power = Math.copySign(0.1, power);
  
          }
  
          elevatorSubsystem.setMotors(power);
  
        } else {
  
          elevatorSubsystem.stallElevator();
  
        }
      } else {
        elevatorSubsystem.stallElevator();
      }
      
      SmartDashboard.putNumber("Elevator Error", error);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    elevatorSubsystem.stallElevator(); 

  }

  public boolean isFinished() {
    
    return -0.2 < error && error < 0.2;

  }
}
