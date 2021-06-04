package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Shoot extends Command {

    private double m_time;

    public Shoot (double time) {
        m_time = time;
        System.out.println("Shooter Constructor");
        requires(Robot.shooter);
    }

    @Override
    protected void initialize() {
        Robot.shooter.setShooterMotors(.75);
        System.out.println("Shooter Init");
        setTimeout(m_time);
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {
        System.out.println("Shooter Finish");
        while(true) {
            if(isTimedOut()) {
                end();
                break;
            }
        }
        return isTimedOut();
    }
    
    @Override
    protected void end() {
        //System.out .println("Shooter End");
        Robot.motorShooter1.set(ControlMode.PercentOutput, 0.0);
        Robot.motorShooter2.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    protected void interrupted() {}
}