// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private CANSparkMax leftMotor = new CANSparkMax(Constants.Elevator.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(Constants.Elevator.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

  public RelativeEncoder leftEncoder = leftMotor.getEncoder();
  public RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final PIDController elevatorController;

  private double PIDOutput = 0;
  private double positionZero = 0;
  public int currentSetpoint;


  public enum ElevatorPositions{
    BASE, MIDDLE, SHOOTAMP,
  }

  public ElevatorSubsystem() {
    elevatorController = new PIDController(Constants.ElevatorConstants.ELEVATOR_KP, 0, 0);
    elevatorController.setIntegratorRange(-0.5, 0.5); //TODO: This must be tuned
    elevatorController.setTolerance(1, 10); //TODO: This must be tuned


    leftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    rightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    

    leftMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ElevatorConstants.ELEVATOR_SETPOINT_SHOOTAMP);
    rightMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ElevatorConstants.ELEVATOR_SETPOINT_SHOOTAMP);

    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    rightMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

  }

  public void setMotors(double speed){
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  public void setElevator(boolean isLifting, double speed){
    if (isLifting){

      setMotors(speed);
    
    }else if (!isLifting){
    
      setMotors(-speed);
    
    }
  }

  public void setSetPoint(double setPoint){

    elevatorController.setSetpoint(setPoint);
  
  }

  public double getSetPoint(){

    return elevatorController.getSetpoint();
  
  }


  public double getPidValue(double setpoint){

    return elevatorController.calculate(leftEncoder.getPosition(), setpoint);

  }
 
  public void pidReset(){

    elevatorController.reset();

  }

  public void stallElevator(){

    rightMotor.set(0.01);
    leftMotor.set(0.01);

  }


  public void setIdleMode(IdleMode mode){

    rightMotor.setIdleMode(mode);
    leftMotor.setIdleMode(mode);

  }

  public double getElevatorPosition() {
    
    return leftEncoder.getPosition();

  }

  public double getEncoder() {
    
    return leftEncoder.getPosition();

  }


  public double getElevatorVelocity(){

    return leftEncoder.getVelocity();

  }

  public void resetElevatorEncoder() {

    leftEncoder.setPosition(0);

  }

  public void setElevatorToPose(double pose){

    leftEncoder.setPosition(pose);

  }

  public void setDistance(double setpoint){

    PIDOutput = elevatorController.calculate(getElevatorPosition(), setpoint);
    setMotors(PIDOutput);

  }

  public boolean atSetpoint(){

    return elevatorController.atSetpoint();
    
  }

  public double getError(){

    return elevatorController.getPositionError();

  }

  // Encoder Methods //
	public void resetEncoder() { 

    positionZero = leftEncoder.getPosition();

}
  public double getEncoderPosition() {  

    return (leftEncoder.getPosition() - positionZero);

}
  public double getEncoderPositionZero() {  

    return positionZero;

}
  public double getEncoderMeters() {

    return leftEncoder.getPosition() * Constants.ElevatorConstants.kEncoderTick2Meter;

}


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator Position:", getElevatorPosition()); 
  
  }
}
