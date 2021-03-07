// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// //import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.buttons.Button;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;
// import frc.robot.commands.Move;
// //import frc.robot.commands.TankDrive;

// /**
//  * This class is the glue that binds the controls on the physical operator
//  * interface to the commands and command groups that allow control of the robot.
//  */
// public class OI {
//   //private Joystick driverController = new Joystick(RobotMap.DRIVER_CONTROLLER);
//  private XboxController driverController = new XboxController(RobotMap.DRIVER_CONTROLLER);
//   Button xButton = new JoystickButton(driverController, RobotMap.Button_X) ;
//   Button bButton = new JoystickButton(driverController, RobotMap.Button_B);
  
//   public double GetDriverRawAxis(int axis){
//     return driverController.getRawAxis(axis);
// // 
//   }
//   public OI() {

//     //xButton.whenPressed(new TankDrive()); // chamge the command to move sequence if you want the whole command to run
//     //bButton.whenPressed(new AutoShoot());

//      }//open driver station and go to 4th button down and find USB order
//   //// CREATING BUTTONS
//   // One type of button is a joystick button which is any button on a
//   //// joystick.
//   // You create one by telling it which joystick it's on and which button
//   // number it is.
//   // Joystick stick = new Joystick(port);
//   // Button button = new JoystickButton(stick, buttonNumber);

//   // There are a few additional built in buttons you can use. Additionally,
//   // by subclassing Button you can create custom triggers and bind those to
//   // commands the same as any other Button.

//   //// TRIGGERING COMMANDS WITH BUTTONS
//   // Once you have a button, it's trivial to bind it to a button in one of
//   // three ways:

//   // Start the command when the button is pressed and let it run the command
//   // until it is finished as determined by it's isFinished method.
//   // button.whenPressed(new ExampleCommand());

//   // Run the command while the button is being held down and interrupt it once
//   // the button is released.
//   // button.whileHeld(new ExampleCommand());

//   // Start the command when the button is released and let it run the command
//   // until it is finished as determined by it's isFinished method.
//   // button.whenReleased(new ExampleCommand());
// }
