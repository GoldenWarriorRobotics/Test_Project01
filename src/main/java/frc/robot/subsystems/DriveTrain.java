// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.manualDrive;

/** Add your docs here. */
public class DriveTrain extends Subsystem {
  private static CANSparkMaxLowLevel motorLeft1 = new CANSparkMax(RobotMap.leftMaster,MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorLeft2 = new CANSparkMax(RobotMap.leftFollower,MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight1 = new CANSparkMax(RobotMap.rightMaster, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight2 = new CANSparkMax(RobotMap.rightFollower,MotorType.kBrushed);

  SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(motorLeft1, motorLeft2);
  SpeedController rightMotorGroup = new SpeedControllerGroup(motorRight1, motorRight2);

  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  public void manualDrive( double move, double turn) {
    drive.arcadeDrive(move, turn);

  
  }
  public void moveDriveTrain(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);


  }
 
//push
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new manualDrive());
  }
}
