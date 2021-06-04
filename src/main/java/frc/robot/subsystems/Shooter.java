package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import java.lang.Thread;

import java.util.concurrent.TimeUnit;
public class Shooter extends Subsystem {

    private VictorSPX motor1 = new WPI_VictorSPX(RobotMap.Motor_Shooter_1);
    private VictorSPX motor2 = new WPI_VictorSPX(RobotMap.Motor_Shooter_2);

    public Shooter(VictorSPX motor1, VictorSPX motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }
   // TimeUnit t = 0;
    @Override
    protected void initDefaultCommand() {
        //leave empty
    }
    
    public void setShooterMotors (double speed) {

        
       
       // t.start();

        motor2.set(ControlMode.PercentOutput, -speed); 
     /*   if(timer > 2.0) {

        }*/
       
        System.out.println("Timer start");
        try {
              TimeUnit t = TimeUnit.SECONDS;
        t.sleep(10);
         } catch (Exception e) {
             System.out.println("Timer Failed");
         }
         System.out.println("Timer stop");

    
        motor1.set(ControlMode.PercentOutput, speed);

        // TODO: Fix later
    }
}