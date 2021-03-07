// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 *
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name.
 * This provides flexibility changing wiring, makes checking the wiring easier and significantly
 * reduces the number of magic numbers floating around.
 */
public class RobotMap {

	public static final int Motor_LEFT_1_ID = 1;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
	public static final int Motor_LEFT_2_ID = 2;
	public static final int Motor_RIGHT_1_ID = 3;
	public static final int Motor_RIGHT_2_ID = 4;
	public static final int DRIVER_CONTROLLER = 0;
	
	public static final int Button_X = 0;
	public static int left_STICK_Y = 1; // check after plugged in the USB cable, see whichever number changes
	public static int right_STICK_Y = 2;// check after plugged in the USB cable, see whichever number changes
	

	
	

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
	// test comment
}
