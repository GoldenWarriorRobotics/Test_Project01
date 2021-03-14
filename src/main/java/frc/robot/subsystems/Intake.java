// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.IntakeDemand;


/** Add your docs here. */
public class Intake extends Subsystem {
  private PWMVictorSPX motor = new PWMVictorSPX(0);// dont know the port value, didnt make a speedcontroller group as there is only motor 


  
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeDemand());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setSpeed(double speed ){
motor.set(speed);
  }
}