// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax RollerTop = new CANSparkMax(Constants.Intake.TOP_ROLLER_ID, MotorType.kBrushless); 
  private final CANSparkMax RollerBottom = new CANSparkMax(Constants.Intake.BOTTOM_ROLLER_ID, MotorType.kBrushless); 

  public IntakeSubsystem() {

    RollerTop.setInverted(false);
    RollerBottom.setInverted(true);
    RollerBottom.setIdleMode(IdleMode.kCoast);
    RollerTop.setIdleMode(IdleMode.kCoast);
  }
  
  public void intakeOn(double power){
    RollerTop.set(-power);
    RollerBottom.set(-power);
  }

  public void intakeShoot(double power){
    RollerTop.set(power);
    RollerBottom.set(power);
  }


  public void setIdleMod(IdleMode idleMode){
    RollerBottom.setIdleMode(idleMode);
    RollerTop.setIdleMode(idleMode);
  }
 

  public double getIntakeSpeed(){
    System.out.println(RollerTop.getAppliedOutput());
    return RollerTop.getAppliedOutput();
  }

  // public void setShooterSpeed(double targetRPM) {
  //   double power = targetRPM / Constants.ShooterSpeedAdjustConstants.MAX_RPM; // Normalize RPM to power (range: 0 to 1)
  //   power = Math.min(1.0, Math.max(-1.0, power)); // Clamp power to valid range
  //   RollerTop.set(power/2);
  //   RollerBottom.set(power/2);
  // }

  public void stopIntake(){
    RollerTop.stopMotor();
    RollerBottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
