// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;
import edu.wpi.first.cameraserver.CameraServer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static CANSparkMaxLowLevel motorLeft1 = new CANSparkMax (RobotMap.leftMaster, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorLeft2 = new CANSparkMax(RobotMap.leftFollower, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight1 = new CANSparkMax(RobotMap.rightMaster, MotorType.kBrushed);
  private static CANSparkMaxLowLevel motorRight2 = new CANSparkMax(RobotMap.rightFollower, MotorType.kBrushed);
  public static DriveTrain driveTrain = new DriveTrain();

  public static VictorSPX motorShooter1 = new VictorSPX(RobotMap.Motor_Shooter_1);
  public static VictorSPX motorShooter2 = new VictorSPX(RobotMap.Motor_Shooter_2);
  public static Shooter shooter = new Shooter(motorShooter1, motorShooter2);

  private Encoder encoder  = new Encoder(0,1,true,EncodingType.k1X);
  private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;
  //private CANEncoder m_encoder;
 // private CANEncoder m_encoder2;  

  public Joystick stick;

  public static OI m_oi;

  /**public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2)); // 2 feet per second
    config.setKinematics(driveTrain.getKinematics());**/

   
    //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(),new Pose2d(1.0,0,new Rotation2d())), config);

   /**  RamseteCommand command = new RamseteCommand(trajectory,driveTrain::getPose,new RamseteController(2.0, 0.7),driveTrain.getFeedforward(),driveTrain.getKinematics(),driveTrain::getSpeeds,
    driveTrain.getLeftPIDController(),driveTrain.getRightPIDController(),driveTrain::setOutput,driveTrain);
    return command;
  }**/


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static frc.robot.subsystems.Intake Intake = new frc.robot.subsystems.Intake();

 // private double startTime;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    CameraServer.getInstance().startAutomaticCapture();
   // motorLeft1 = new CANSparkMax(RobotMap.leftMaster, MotorType.kBrushed);
    //motorRight1 = new CANSparkMax(RobotMap.rightMaster, MotorType.kBrushed);
    
    //motorLeft1.restoreFactoryDefaults();
    //motorRight2.restoreFactoryDefaults();

  //  m_encoder = motorLeft1.getEncoder(EncoderType.kQuadrature,4096);
   // m_encoder2 = motorRight1.getEncoder(EncoderType.kQuadrature, 4096);

    
    
    //container = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get()*kDriveTick2Feet);
   // SmartDashboard.putNumber("Left Drive Encoder Value", m_encoder.getPosition());
    //SmartDashboard.putNumber("Right Drive Encoder Value", m_encoder2.getPosition());
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset
   * any subsystem information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the chooser code above
   * (like the commented example) or additional comparisons to the switch structure below with
   * additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    //startTime = Timer.getFPGATimestamp();
    encoder.reset();
    errorSum = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError =0;

    // ok motorLeft1.restoreFactoryDefaults();
    // ok  motorRight2.restoreFactoryDefaults();
    //Container.getAutonomousCommand().schedule();
   // .set(.3);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.start();
    //}
  }
  double setPoint =0;
  double errorSum = 0;
  double lastTimeStamp = 0;
  final  double kP = 0.05;
  final double kI = 0.5;
  final double iLimit = 1;
  final double kD = 0.01;
   double lastError =0;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //if(stick.getRawButton(7)) {
      setPoint = 10;
    //} else if (stick.getRawButton(8)) {
      setPoint = 0;
    //}
    //get sensor posistion

    double sensorPosition =encoder.get() * kDriveTick2Feet;

    //calculations
    double error = setPoint -sensorPosition;
    double outputSpeed = kP * error;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    if(Math.abs(error) <iLimit) {
      errorSum += error *dt;
    }

    double errorRate = (error - lastError) / dt;
    double outSpeed = kP * error +kI * errorSum +kD *errorRate;
    //output to motors
    motorLeft1.set(outSpeed);
    motorLeft2.set(outSpeed);
    motorRight1.set(-outSpeed);
    motorRight2.set(-outSpeed); 

    //update last- variables
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
  }
   /**  double time = Timer.getFPGATimestamp();
    if (time -startTime <3) {
    motorLeft1.set(.6);
    motorLeft2.set(.6);
    motorRight1.set(.6);
    motorRight2.set(.6);
    } else{
      motorLeft1.set(0);
      motorLeft2.set(0);
      motorRight1.set(0);
      motorRight2.set(0);

    }

    double leftPosition = m_encoder.getPosition() * kDriveTick2Feet;
    double rightPosition = m_encoder2.getPosition()*kDriveTick2Feet;
    double distance = (leftPosition+rightPosition)/2;

    if(distance < 10){
      driveTrain.moveDriveTrain(0.6,0.6);


    }else {
      driveTrain.moveDriveTrain(0, 0);        
    }
   
    Scheduler.getInstance().run();
  }**/

  @Override
  public void teleopInit() {


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    
    }
    driveTrain.moveDriveTrain(.3, .3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
