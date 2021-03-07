// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
//import frc.robot.commands.TankDrive;

/** Add your docs here. */
public class DriveTrain extends Subsystem {
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;
  
  
  public DriveTrain(CANSparkMax motorLeft1, CANSparkMax motorLeft2, CANSparkMax motorRight1, CANSparkMax motorRight2 ){
   this.motorLeft1 = motorLeft1;
   this.motorLeft2 = motorLeft2;
   this.motorRight1 = motorRight1;
   this.motorRight2 = motorRight2;
  }


  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new TankDrive());
    
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setLeftMotors(double speed) {
    motorLeft2.set(-speed);
    motorLeft1.set(-speed);
  }
  public void setRightMotors(double speed) {
    motorRight1.set(speed);
    motorRight2.set(speed);
  }
}
