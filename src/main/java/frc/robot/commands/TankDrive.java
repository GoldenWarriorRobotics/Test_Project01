// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;
// import frc.robot.RobotMap;

// public class TankDrive extends Command {
//   public TankDrive() {

//     // Use requires() here to declare subsystem dependencies

//     requires(Robot.driveTrain);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {}

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() { //called everytime through teleop periodic
//   double leftStickY = Robot.m_oi.GetDriverRawAxis(RobotMap.left_STICK_Y); //getting the values, if you want to move stright have to move both sticks forward
//   double rightStickY = Robot.m_oi.GetDriverRawAxis(RobotMap.right_STICK_Y);
//    // plug in joystick and see which of the letters change, that is your number
//    Robot.driveTrain.setLeftMotors(leftStickY * 0.5);
//    Robot.driveTrain.setRightMotors(rightStickY * 0.5) ;
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//     Robot.driveTrain.setLeftMotors(0);
//     Robot.driveTrain.setRightMotors(0);
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//     this.end();
//   }
// }
