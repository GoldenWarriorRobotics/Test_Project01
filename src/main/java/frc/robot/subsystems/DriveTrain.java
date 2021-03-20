// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.RobotMap;
import frc.robot.commands.manualDrive;

/** Add your docs here. */
public class DriveTrain extends Subsystem {
  private static CANSparkMaxLowLevel motorLeft1 = new CANSparkMax(RobotMap.leftMaster, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorLeft2 = new CANSparkMax(RobotMap.leftFollower, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight1 = new CANSparkMax(RobotMap.rightMaster, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight2 = new CANSparkMax(RobotMap.rightFollower, MotorType.kBrushed);


  private static 
 // public Rotation2d getHeading() {
 //   return Rotation2d.fromDegrees(-m_gyro.getAngle());
 // }

  // Gyro gyro = new ADXRS450_Gyro(); //put in the port the GYro is connected to
  // on the robot
  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  //DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //width of robot 
  //DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

 // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.89, .243); //put in the correct ks and kv values
 //PIDController leftPidController = new PIDController(9.95, 0, 0);
 //PIDController rightPidController = new PIDController(9.95, 0, 0);
 

  SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(motorLeft1, motorLeft2);
  SpeedController rightMotorGroup = new SpeedControllerGroup(motorRight1, motorRight2);

  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  public void manualDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);

  }

  public void moveDriveTrain(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);

  }

  // push
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new manualDrive());
  }
}

  //Encoder m_leftEncoder = new Encoder(0, 1);
  ////Encoder m_rightEncoder = new Encoder(0, 1);

//public SimpleMotorFeedforward getFeedforward(){
 // return feedforward;
//}
//public PIDController getLeftPIDController(){
 // return leftPidController;
//}

//public PIDController getRightPIDController(){
  //return rightPidController;
//}

/**public DifferentialDriveKinematics getKinematics() {
  return kinematics;
}
Pose2d pose;

@Override
public void periodic() {
  Pose2d pose = odometry.update(getHeading(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
}



public Pose2d getPose(){
  return pose;
}
public void setOutput(double leftVoltage, double rightVoltage){
  motorLeft1.set(leftVoltage/12);
  motorLeft2.set(rightVoltage/12);
}

}**/
