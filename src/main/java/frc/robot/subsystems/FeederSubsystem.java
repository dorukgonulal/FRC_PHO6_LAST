// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotID;

public class FeederSubsystem extends SubsystemBase {

  private final CANSparkMax RollerBack = new CANSparkMax(Constants.Intake.BACK_ROLLER_ID, MotorType.kBrushless);

  public FeederSubsystem() {

    RollerBack.setInverted(true);
  }
  
 

  public void feedOn(double power){
    RollerBack.set(-power);
  }

  public void feedShoot(double power){
    RollerBack.set(power);
  }




  public void feedStop(){
    RollerBack.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
