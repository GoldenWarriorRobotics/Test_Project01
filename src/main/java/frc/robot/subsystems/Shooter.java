package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Shooter extends Subsystem {

    private VictorSPX motor1 = new WPI_VictorSPX(RobotMap.Motor_Shooter_1);
    private VictorSPX motor2 = new WPI_VictorSPX(RobotMap.Motor_Shooter_2);

    public Shooter(VictorSPX motor1, VictorSPX motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    @Override
    protected void initDefaultCommand() {
        //leave empty
    }
    
    public void setShooterMotors (double speed) {
        motor1.set(ControlMode.PercentOutput, speed);
        motor2.set(ControlMode.PercentOutput, -speed);
    }
}