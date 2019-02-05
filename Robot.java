/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// A project by Ricardo Santamaria-Sarcos, Dario Zaccagnino, Christian Kolowich and David James

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
//Define Motors Controllers
//DRIVE TRAIN
  //Right Side Motors
  TalonSRX  rightMasterMotor1 = new TalonSRX  (3);
  VictorSPX rightSlaveMotor2  = new VictorSPX (5);
  VictorSPX rightSlaveMotor3  = new VictorSPX (7);
  //Left Side Motors
  TalonSRX  leftMasterMotor1  = new TalonSRX  (2);
  VictorSPX leftSlaveMotor2   = new VictorSPX (4);
  VictorSPX leftSlaveMotor3   = new VictorSPX (6);
//ELEVATOR
  //Left GB
  TalonSRX  elevLeftMaster    = new TalonSRX  (8);
  VictorSPX elevLeftSlave     = new VictorSPX (9);
  //Right GB
  TalonSRX  elevRightMaster   = new TalonSRX  (10);
  VictorSPX elevRightSlave    = new VictorSPX (11);
//WRIST
  TalonSRX  wristMaster       = new TalonSRX  (12);
  VictorSPX wristSlave        = new VictorSPX (13);
  //Joysticks
  private Joystick leftJoy;
  private Joystick rghtJoy;
  private Joystick operatorJoy;
 
  //Usually Variables
  //Encoder Counts per Revolution
  final private double countPerRev = 4096;
  //wheel Radius
  final private double wheelRadius = 3;
  //Wheel Circumference
  final private double wheelCircumference = 2* Math.PI * wheelRadius; //Circumference (in inches) (2*r*pi)
  double distance;
  double velocity; 
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    //Joystick
    leftJoy = new Joystick(0);
    rghtJoy = new Joystick(1);  
    operatorJoy = new Joystick(2);  
    

      //DRIVE TRAIN
      //Sets up motor controller settings for right side
      rightMasterMotor1.setInverted(false);
      rightSlaveMotor2.setInverted(false);
      rightSlaveMotor3.setInverted(false);
      rightMasterMotor1.configOpenloopRamp(1, 0);
      rightMasterMotor1.setNeutralMode(NeutralMode.Brake);
      rightSlaveMotor2.setNeutralMode(NeutralMode.Brake);
      rightSlaveMotor3.setNeutralMode(NeutralMode.Brake);
      
      //Encoder Right Side
      rightMasterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.kTimeoutMs);
      //Reverse Direction of Encoder
      rightMasterMotor1.setSensorPhase(true);
      
      //Config PID F Gains
      rightMasterMotor1.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
      rightMasterMotor1.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
      rightMasterMotor1.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
      rightMasterMotor1.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
      
      
      //Sets up motor controller settings for left side
      leftMasterMotor1.setInverted(true);
      leftSlaveMotor2.setInverted(true);
      leftSlaveMotor3.setInverted(true);
      leftMasterMotor1.configOpenloopRamp(1, 0);
      leftMasterMotor1.setNeutralMode(NeutralMode.Brake);
      leftSlaveMotor2.setNeutralMode(NeutralMode.Brake);
      leftSlaveMotor3.setNeutralMode(NeutralMode.Brake);
      //Encoder Left Side
      leftMasterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.kTimeoutMs);
      //Reverse Direction of Encoder
      leftMasterMotor1.setSensorPhase(true);

      //Config PID F Gains
      leftMasterMotor1.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
      leftMasterMotor1.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
      leftMasterMotor1.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
      leftMasterMotor1.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
    //END OF DRIVE TRAIN

    //ELEVATOR
    //Left
    elevLeftMaster.setInverted(true); //Reverse direction
    elevLeftSlave .setInverted(true); //Reverse direction
    elevLeftMaster.setNeutralMode(NeutralMode.Brake);
    elevLeftSlave .setNeutralMode(NeutralMode.Brake);
      //Encoder
      elevLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.kTimeoutMs);
      elevLeftMaster.setSensorPhase(true); //Reverse direction
      //Config PID F Gains
      elevLeftMaster.config_kP(Constants.kSlot_Elev, Constants.kGains_Elev.kP, Constants.kTimeoutMs);
      elevLeftMaster.config_kI(Constants.kSlot_Elev, Constants.kGains_Elev.kI, Constants.kTimeoutMs);
      elevLeftMaster.config_kD(Constants.kSlot_Elev, Constants.kGains_Elev.kD, Constants.kTimeoutMs);
      elevLeftMaster.config_kF(Constants.kSlot_Elev, Constants.kGains_Elev.kF, Constants.kTimeoutMs);
      elevLeftMaster.configMotionAcceleration(Constants.kElevAccel, Constants.kTimeoutMs);
      elevLeftMaster.configMotionCruiseVelocity(Constants.kElevVel, Constants.kTimeoutMs);
    //Right
    elevRightMaster.setInverted(false); //Reverse direction
    elevRightSlave .setInverted(false); //Reverse direction
    elevRightMaster.setNeutralMode(NeutralMode.Brake);
    elevRightSlave .setNeutralMode(NeutralMode.Brake);
      //Encoder
      elevRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.kTimeoutMs);
      elevRightMaster.setSensorPhase(false); //Reverse direction
      //Config PID F Gains
      elevRightMaster.config_kP(Constants.kSlot_Elev, Constants.kGains_Elev.kP, Constants.kTimeoutMs);
      elevRightMaster.config_kI(Constants.kSlot_Elev, Constants.kGains_Elev.kI, Constants.kTimeoutMs);
      elevRightMaster.config_kD(Constants.kSlot_Elev, Constants.kGains_Elev.kD, Constants.kTimeoutMs);
      elevRightMaster.config_kF(Constants.kSlot_Elev, Constants.kGains_Elev.kF, Constants.kTimeoutMs);
      elevRightMaster.configMotionAcceleration(Constants.kElevAccel, Constants.kTimeoutMs);
      elevRightMaster.configMotionCruiseVelocity(Constants.kElevVel, Constants.kTimeoutMs);
    //END OF ELEVATOR
    
    //WRIST

    //END OF WRIST

    //INTAKE

    //END OF INTAKE
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here 
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
      double rightEncoderPos = rightMasterMotor1.getSelectedSensorPosition(0);
      double rightEncoderVel = rightMasterMotor1.getSelectedSensorVelocity(0);
      //double leftEncoderPos = leftMasterMotor1.getSelectedSensorPosition();
      //Encoder Counts per Distance
    
      //double cimRPM = 4500;
      //double gearRatio = 6.23;
      
    
      //velocity = (cimRPM/600)*(countPerRev/gearRatio);
      distance = (rightEncoderPos * wheelCircumference)/countPerRev; 
      
 
      SmartDashboard.putNumber("Right Side Pos", distance);
      SmartDashboard.putNumber("Right Side Vel", rightEncoderVel);
      //SmartDashboard.putNumber("Max Velocity", velocity);
      //System.out.println("Right Side Encoder Postion: " + distance);
    
    //Drive Train Controls
      //Control Mode - Right side controlled by right joystick
      rightMasterMotor1.set(ControlMode.PercentOutput, rghtJoy.getY());
      //Follow Master
      rightSlaveMotor2.follow(rightMasterMotor1);
      rightSlaveMotor3.follow(rightMasterMotor1);

      //Control Mode - Left side conrolled by left joystick
      leftMasterMotor1.set(ControlMode.PercentOutput, leftJoy.getY());
      //Follow Master
      leftSlaveMotor2.follow(leftMasterMotor1);
      leftSlaveMotor3.follow(leftMasterMotor1);
     
      //move elev up 50 inches
      if(rghtJoy.getTrigger()){
        MoveElevToTarget(50.0);
      } else {
        elevLeftMaster.set(ControlMode.PercentOutput, operatorJoy.getY());
        elevRightMaster.set(ControlMode.PercentOutput, operatorJoy.getY());
        elevLeftSlave.follow(elevLeftMaster);
        elevRightSlave.follow(elevRightMaster);
      }
/*
      //Intake Mode - 1 Motor controlled but a Trigger
      if(secondJoy.getTriggerPressed()){
         intakeMotor.set(ControlMode.PercentOutput, .5);
      }

      //Outtake Mode with button number 3
      if(secondJoy.getRawButtonPressed(5)){
        intakeMotor.set(ControlMode.PercentOutput, -.5);
      }

      //Arm Mode up - Controlled by a buttons 3
      if(secondJoy.getRawButtonPressed(3)){
      armMasterMotor.set(ControlMode.PercentOutput, .5);
      }
      //Arm Mode down - controlled by a button 2
      if(secondJoy.getRawButtonPressed(2)){
      armMasterMotor.set(ControlMode.PercentOutput, -.5);
      }
      //Follow Master
      armSlaveMotor.follow(armMasterMotor);

      //Elevator - Elevator controlled by a joystick
      elevatorMasterMotor.set(ControlMode.PercentOutput,secondJoy.getY());
      //Follow Master
      elevatorSlaveMotor.follow(elevatorMasterMotor);
      */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }


  //Drive set amount  - in inches
  public void driveToTargetDist(double dist){
    double targetDist = dist; //Inches
    double setPoint;
    setPoint = (dist * countPerRev)/wheelCircumference;
    System.out.println("Button Pressed. Moving to: " + targetDist);
    rightMasterMotor1.set(ControlMode.Position, setPoint);
  }

  //Move Elev set amount  - in inches
  public void MoveElevToTarget(double dist){
    double targetDist = dist; //Inches
    double setPoint;
    setPoint = (dist * countPerRev)/wheelCircumference;
    System.out.println("Button Pressed. Moving to: " + targetDist);
    elevRightMaster.set(ControlMode.MotionMagic, setPoint);
    elevRightSlave.follow(elevRightMaster);
    elevLeftMaster.set(ControlMode.MotionMagic, setPoint);
    elevLeftSlave.follow(elevLeftMaster);
  }
}
