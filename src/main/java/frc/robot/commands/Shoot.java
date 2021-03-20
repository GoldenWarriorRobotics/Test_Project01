package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Shoot extends Command {

    private int m_time;
    private double m_speed;

    public Shoot () {
        requires(Robot.shooter);
    }
    
    public Shoot (int time, double speed) {
        m_time = time;
        m_speed = speed;
        requires(Robot.shooter);
    }

    @Override
    protected void initialize() {
        Robot.shooter.setShooterMotors(m_speed);
        setTimeout(m_time);
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {        
        return isTimedOut();
    }
    
    @Override
    protected void end() {
        Robot.motorShooter1.set(ControlMode.PercentOutput, 0.0);
        Robot.motorShooter2.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    protected void interrupted() {}
}