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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;




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
    public TalonSRX  rightMasterMotor1 = new TalonSRX  (3);
    public VictorSPX rightSlaveMotor2  = new VictorSPX (5);
    public VictorSPX rightSlaveMotor3  = new VictorSPX (7);
    //Left Side Motors
    public TalonSRX  leftMasterMotor1  = new TalonSRX  (2);
    public VictorSPX leftSlaveMotor2   = new VictorSPX (4);
    public VictorSPX leftSlaveMotor3   = new VictorSPX (6);
  //ELEVATOR
    //Left GB
    public static TalonSRX  elevLeftMaster    = new TalonSRX  (8);
    public static VictorSPX elevLeftSlave     = new VictorSPX (11);
    //Right GB
    public static TalonSRX  elevRightMaster   = new TalonSRX  (9);
    public static VictorSPX elevRightSlave    = new VictorSPX (10);
  //WRIST
    public static TalonSRX  wristMaster       = new TalonSRX  (13);
    public static VictorSPX wristSlave        = new VictorSPX (12);
  //INTAKE
    public static VictorSPX intakeMotor       = new VictorSPX (14);
  //Climbing
    public static TalonSRX  climbMotorMaster  = new TalonSRX(15);
    public static VictorSPX climbMotorSlave   = new VictorSPX(16);
  //Joysticks
    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static XboxController operatorController;
    /**
     * A Button - 1
     * B Button - 2 
     * X button - 3
     * Y Button - 4
     * Left Bumper - 5
     * Right Bumber - 6
     * Select Button - 7
     * Start Button - 8
     */
    JoystickButton aButton      = new JoystickButton(operatorController, 1);
    JoystickButton bButton      = new JoystickButton(operatorController, 2);
    JoystickButton xButton      = new JoystickButton(operatorController,3);
    JoystickButton yButton      = new JoystickButton(operatorController, 4);
    JoystickButton LeftBumper   = new JoystickButton(operatorController, 5);
    JoystickButton RightBumper  = new JoystickButton(operatorController, 6);
    JoystickButton rightTrigger = new JoystickButton(rightJoy, 1);    
    JoystickButton button2      = new JoystickButton(rightJoy, 2);                                                                                                                                                          



  //Usually Variables
    //Encoder Counts per Revolution
    final private double countPerRev = 4096;
    //wheel Radius
    final private double wheelRadius = 3;
    //Wheel Circumference
    final private double wheelCircumference = 2 * Math.PI * wheelRadius; //Circumference (in inches) (2*r*pi)
    final public static double sprocketPitchCircumference =  1.79 * Math.PI;
    final private double armRatio = 44/18;
    double distance;
    double velocity; 
    //Gyro
    private ADXRS450_Gyro onboardGyro;
    //Limelight
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    //LED - Test to see if the light is on/off during auto?
    boolean ledStatus = true;

    //Wrist Commands
    MoveWrist wristIntake  = new MoveWrist(Constants.wristDown);
    MoveWrist wristRest    = new MoveWrist(Constants.wristStraight);
    //Elevator Commands
    MoveElev elevBot       = new MoveElev(Constants.elevBotGoal);
    MoveElev elevLow       = new MoveElev(Constants.elevLowGoal);
    MoveElev elevMid       = new MoveElev(Constants.elevMidGoal);
    MoveElev elevTop       = new MoveElev(Constants.elevTopGoal);
    //Climb Commands
    MoveClimb extendClimb  = new MoveClimb(Constants.climbExtend);
    MoveClimb retractClimb = new MoveClimb(Constants.climbRetract);



    
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    //CONTROLLERS
      leftJoy            = new Joystick(0);
      rightJoy           = new Joystick(1);
      operatorController = new XboxController(2); 
    //END OF CONTROLLERS
  
    //GYRO
      onboardGyro = new  ADXRS450_Gyro();
      onboardGyro.calibrate();//Calibrates the gyro
      onboardGyro.reset();//Sets gyro to 0 degrees
    //END OF GYRO
    
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
    //END OF DRIVE TRAIN

    //ELEVATOR
      //Left
      elevLeftMaster.setInverted(true); //Reverse direction
      elevLeftSlave .setInverted(true); //Reverse direction
      elevLeftMaster.setNeutralMode(NeutralMode.Brake);
      elevLeftSlave .setNeutralMode(NeutralMode.Brake);
        //Encoder
        elevLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY, Constants.kTimeoutMs);
        elevLeftMaster.setSensorPhase(true); //Reverse direction

        //Config PID F Gains
        elevLeftMaster.config_kP(Constants.kSlot_Elev, Constants.kGains_Elev.kP, Constants.kTimeoutMs);
        elevLeftMaster.config_kI(Constants.kSlot_Elev, Constants.kGains_Elev.kI, Constants.kTimeoutMs);
        elevLeftMaster.config_kD(Constants.kSlot_Elev, Constants.kGains_Elev.kD, Constants.kTimeoutMs);
        elevLeftMaster.config_kF(Constants.kSlot_Elev, Constants.kGains_Elev.kF, Constants.kTimeoutMs);
        elevLeftMaster.configClosedLoopPeakOutput(Constants.kSlot_Elev, Constants.kGains_Elev.kPeakOutput);
        elevLeftMaster.configMotionAcceleration(Constants.kElevAccel, Constants.kTimeoutMs);
        elevLeftMaster.configMotionCruiseVelocity(Constants.kElevVel, Constants.kTimeoutMs);
      //Right
      elevRightMaster.setInverted(false); //Reverse direction
      elevRightSlave .setInverted(true); //Reverse direction
      elevRightMaster.setNeutralMode(NeutralMode.Brake);
      elevRightSlave .setNeutralMode(NeutralMode.Brake);
        //Encoder
        elevRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY, Constants.kTimeoutMs);
        elevRightMaster.setSensorPhase(false); //Reverse direction
        elevRightMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
        //Config PID F Gains
        elevRightMaster.config_kP(Constants.kSlot_Elev, Constants.kGains_Elev.kP, Constants.kTimeoutMs);
        elevRightMaster.config_kI(Constants.kSlot_Elev, Constants.kGains_Elev.kI, Constants.kTimeoutMs);
        elevRightMaster.config_kD(Constants.kSlot_Elev, Constants.kGains_Elev.kD, Constants.kTimeoutMs);
        elevRightMaster.config_kF(Constants.kSlot_Elev, Constants.kGains_Elev.kF, Constants.kTimeoutMs);
        elevRightMaster.configClosedLoopPeakOutput(Constants.kSlot_Elev, Constants.kGains_Elev.kPeakOutput);
        elevRightMaster.configMotionAcceleration(Constants.kElevAccel, Constants.kTimeoutMs);
        elevRightMaster.configMotionCruiseVelocity(Constants.kElevVel, Constants.kTimeoutMs);
    //END OF ELEVATOR
    
    //WRIST
      //Left
      wristMaster.setInverted(false); //Reverse direction
      wristSlave .setInverted(true); //Reverse direction
      wristMaster.setNeutralMode(NeutralMode.Brake);
      wristSlave .setNeutralMode(NeutralMode.Brake);
        //Encoder
        wristMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY, Constants.kTimeoutMs);
        wristMaster.setSensorPhase(false); //Reverse direction
        //Config PID F Gains
        wristMaster.config_kP(Constants.kSlot_Wrist, Constants.kGains_Wrist.kP, Constants.kTimeoutMs);
        wristMaster.config_kI(Constants.kSlot_Wrist, Constants.kGains_Wrist.kI, Constants.kTimeoutMs);
        wristMaster.config_kD(Constants.kSlot_Wrist, Constants.kGains_Wrist.kD, Constants.kTimeoutMs);
        wristMaster.config_kF(Constants.kSlot_Wrist, Constants.kGains_Wrist.kF, Constants.kTimeoutMs);
        wristMaster.configClosedLoopPeakOutput(Constants.kSlot_Wrist, Constants.kGains_Wrist.kPeakOutput);
        wristMaster.configMotionAcceleration(Constants.kWristAccel, Constants.kTimeoutMs);
        wristMaster.configMotionCruiseVelocity(Constants.kWristVel, Constants.kTimeoutMs);
    //END OF WRIST

    //INTAKE
      intakeMotor.setInverted(true);
      intakeMotor.setNeutralMode(NeutralMode.Brake);
    //END OF INTAKE

    //Climbing
      climbMotorMaster.setInverted(false);
      climbMotorSlave.setInverted(false);
      climbMotorMaster.setNeutralMode(NeutralMode.Brake);
      climbMotorSlave.setNeutralMode(NeutralMode.Brake);
        //Encoder
        climbMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY, Constants.kTimeoutMs);
        climbMotorMaster.setSensorPhase(false);
        //Config PID F Gains
        climbMotorMaster.config_kP(Constants.kSlot_Climb, Constants.kGains_Climb.kP, Constants.kTimeoutMs);
        climbMotorMaster.config_kI(Constants.kSlot_Climb, Constants.kGains_Climb.kI, Constants.kTimeoutMs);
        climbMotorMaster.config_kD(Constants.kSlot_Climb, Constants.kGains_Climb.kD, Constants.kTimeoutMs);
        climbMotorMaster.config_kF(Constants.kSlot_Climb, Constants.kGains_Climb.kF, Constants.kTimeoutMs);
        climbMotorMaster.configClosedLoopPeakOutput(Constants.kSlot_Climb, Constants.kGains_Climb.kPeakOutput);
        climbMotorMaster.configMotionAcceleration(Constants.kClimbAccel, Constants.kTimeoutMs);
        climbMotorMaster.configMotionCruiseVelocity(Constants.kClimbVel, Constants.kTimeoutMs);

    //END OF CLIMBING
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
      Scheduler.getInstance().run();
      SmartDashboard.putNumber("Wrist Postion (encoder ticks): ", wristMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
      SmartDashboard.putNumber("Elev R Postion (encoder ticks): ", elevRightMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
      SmartDashboard.putNumber("Elev L Postion (encoder ticks): ", elevLeftMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
      Update_Limelight_Tracking();
      double driveForwardPower;
      double turnPower;
      if(wristIntake.isRunning() || wristRest.isRunning()){
  
      } else {
        wristMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kRight));
        wristSlave.follow(wristMaster);
      }
  
      if(elevBot.isRunning() || elevLow.isRunning() || elevMid.isRunning() || elevTop.isRunning()){
  
      } else {
        elevLeftMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kLeft));
        elevLeftSlave.follow(elevLeftMaster);
        elevRightMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kLeft));
        elevRightSlave.follow(elevRightMaster);
      }
  
      //Drive Train Controls
      if (leftJoy.getTrigger()){
        if (m_LimelightHasValidTarget){
          //Limelight vision procesing control
          SmartDashboard.putString("Valid Target", "True");
          driveForwardPower = m_LimelightDriveCommand;
          turnPower = m_LimelightSteerCommand;
          rightMasterMotor1.set(ControlMode.PercentOutput, driveForwardPower+turnPower);
          rightSlaveMotor2.follow(rightMasterMotor1);
          rightSlaveMotor3.follow(rightMasterMotor1);
          leftMasterMotor1.set(ControlMode.PercentOutput, driveForwardPower-turnPower);
          leftSlaveMotor2.follow(leftMasterMotor1);
          leftSlaveMotor3.follow(leftMasterMotor1);
        } else {
          SmartDashboard.putString("Valid Target", "False");
        }
      } else {
        //Arcade Drive
        //Right
        rightMasterMotor1.set(ControlMode.PercentOutput, -rightJoy.getY());
        rightSlaveMotor2.follow(rightMasterMotor1);
        rightSlaveMotor3.follow(rightMasterMotor1);
        //Left
        leftMasterMotor1.set(ControlMode.PercentOutput, -leftJoy.getY());
        leftSlaveMotor2.follow(leftMasterMotor1);
        leftSlaveMotor3.follow(leftMasterMotor1);
      }
      /**
       * A Button - 1 - elevator Bottom Position
       * B Button - 2  - elevator Low Position
       * X button - 3 - elevator Mid Position
       * Y Button - 4 - elevator Top Position
       * Left Bumper - 5 - Wrist down to intake
       * Right Bumber - 6 - Wrist up to resting position
       * Select Button - 7
       * Start Button - 8
       */
      //Active
      LeftBumper.whenPressed(wristIntake);
      RightBumper.whenPressed(wristRest);
      aButton.whenPressed(elevBot);
      bButton.whenPressed(elevLow);
      xButton.whenPressed(elevMid);
      yButton.whenPressed(elevTop);
      rightTrigger.whenPressed(extendClimb);
      button2.whenPressed(retractClimb);
      //DeActivate
      LeftBumper.cancelWhenPressed(wristRest);
      RightBumper.cancelWhenPressed(wristIntake);
      aButton.cancelWhenPressed(elevLow);
      aButton.cancelWhenPressed(elevMid);
      aButton.cancelWhenPressed(elevTop);
      bButton.cancelWhenPressed(elevBot);
      bButton.cancelWhenPressed(elevMid);
      bButton.cancelWhenPressed(elevTop);
      xButton.cancelWhenPressed(elevBot);
      xButton.cancelWhenPressed(elevLow);
      xButton.cancelWhenPressed(elevTop);
      yButton.cancelWhenPressed(elevBot);
      yButton.cancelWhenPressed(elevLow);
      yButton.cancelWhenPressed(elevMid);
      rightTrigger.cancelWhenPressed(retractClimb);
      button2.cancelWhenPressed(extendClimb);
  
      //Reset Gyro
      if(leftJoy.getRawButton(6)){
        onboardGyro.reset();
      }
  
      // turn on/off Vision Tracking
      if(leftJoy.getRawButtonPressed(7)){
        if(ledStatus){ //turn off
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
          ledStatus = false;
        } else { //turn on
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
          NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
          ledStatus = true;
        }
      }
      
      //Intake
      if(leftJoy.getRawButton(2)){
        intakeMotor.set(ControlMode.PercentOutput, 0.75);
      } else if(leftJoy.getRawButton(3)){
        intakeMotor.set(ControlMode.PercentOutput, -0.5);
      } else {
        intakeMotor.set(ControlMode.PercentOutput, 0.2);
      }
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Wrist Postion (encoder ticks): ", wristMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("Elev R Postion (encoder ticks): ", elevRightMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("Elev L Postion (encoder ticks): ", elevLeftMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
    Update_Limelight_Tracking();
    double driveForwardPower;
    double turnPower;
    if(wristIntake.isRunning() || wristRest.isRunning()){

    } else {
      wristMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kRight));
      wristSlave.follow(wristMaster);
    }

    if(elevBot.isRunning() || elevLow.isRunning() || elevMid.isRunning() || elevTop.isRunning()){

    } else {
      elevLeftMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kLeft));
      elevLeftSlave.follow(elevLeftMaster);
      elevRightMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kLeft));
      elevRightSlave.follow(elevRightMaster);
    }

    //Drive Train Controls
    if (leftJoy.getTrigger()){
      if (m_LimelightHasValidTarget){
        //Limelight vision procesing control
        SmartDashboard.putString("Valid Target", "True");
        driveForwardPower = m_LimelightDriveCommand;
        turnPower = m_LimelightSteerCommand;
        rightMasterMotor1.set(ControlMode.PercentOutput, driveForwardPower+turnPower);
        rightSlaveMotor2.follow(rightMasterMotor1);
        rightSlaveMotor3.follow(rightMasterMotor1);
        leftMasterMotor1.set(ControlMode.PercentOutput, driveForwardPower-turnPower);
        leftSlaveMotor2.follow(leftMasterMotor1);
        leftSlaveMotor3.follow(leftMasterMotor1);
      } else {
        SmartDashboard.putString("Valid Target", "False");
      }
    } else {
      //Arcade Drive
      //Right
      rightMasterMotor1.set(ControlMode.PercentOutput, -rightJoy.getY());
      rightSlaveMotor2.follow(rightMasterMotor1);
      rightSlaveMotor3.follow(rightMasterMotor1);
      //Left
      leftMasterMotor1.set(ControlMode.PercentOutput, -leftJoy.getY());
      leftSlaveMotor2.follow(leftMasterMotor1);
      leftSlaveMotor3.follow(leftMasterMotor1);
    }
    /**
     * A Button - 1 - elevator Bottom Position
     * B Button - 2  - elevator Low Position
     * X button - 3 - elevator Mid Position
     * Y Button - 4 - elevator Top Position
     * Left Bumper - 5 - Wrist down to intake
     * Right Bumber - 6 - Wrist up to resting position
     * Select Button - 7
     * Start Button - 8
     */
    //Active
    LeftBumper.whenPressed(wristIntake);
    RightBumper.whenPressed(wristRest);
    aButton.whenPressed(elevBot);
    bButton.whenPressed(elevLow);
    xButton.whenPressed(elevMid);
    yButton.whenPressed(elevTop);
    rightTrigger.whenPressed(extendClimb);
    button2.whenPressed(retractClimb);
    //DeActivate
    LeftBumper.cancelWhenPressed(wristRest);
    RightBumper.cancelWhenPressed(wristIntake);
    aButton.cancelWhenPressed(elevLow);
    aButton.cancelWhenPressed(elevMid);
    aButton.cancelWhenPressed(elevTop);
    bButton.cancelWhenPressed(elevBot);
    bButton.cancelWhenPressed(elevMid);
    bButton.cancelWhenPressed(elevTop);
    xButton.cancelWhenPressed(elevBot);
    xButton.cancelWhenPressed(elevLow);
    xButton.cancelWhenPressed(elevTop);
    yButton.cancelWhenPressed(elevBot);
    yButton.cancelWhenPressed(elevLow);
    yButton.cancelWhenPressed(elevMid);
    rightTrigger.cancelWhenPressed(retractClimb);
    button2.cancelWhenPressed(extendClimb);

    //Reset Gyro
    if(leftJoy.getRawButton(6)){
      onboardGyro.reset();
    }

    // turn on/off Vision Tracking
    if(leftJoy.getRawButtonPressed(7)){
      if(ledStatus){ //turn off
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        ledStatus = false;
      } else { //turn on
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        ledStatus = true;
      }
    }
    
    //Intake
    if(leftJoy.getRawButton(2)){
      intakeMotor.set(ControlMode.PercentOutput, 0.75);
    } else if(leftJoy.getRawButton(3)){
      intakeMotor.set(ControlMode.PercentOutput, -0.5);
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0.2);
    }
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
    setPoint = (dist * countPerRev)/sprocketPitchCircumference;
    System.out.println("Button Pressed. Moving to: " + targetDist);
    elevRightMaster.set(ControlMode.MotionMagic, setPoint);
    elevRightSlave.follow(elevRightMaster);
    elevLeftMaster.set(ControlMode.MotionMagic, setPoint);
    elevLeftSlave.follow(elevLeftMaster);
  }
  //Move Elev set amount  - in talon native units
  public void MoveElevToRawPosition(double position){
    elevRightMaster.set(ControlMode.MotionMagic, position);
    elevRightSlave.follow(elevRightMaster);
    elevLeftMaster.set(ControlMode.MotionMagic, position);
    elevLeftSlave.follow(elevLeftMaster);
  }

  //Move wrist to raw postion - in degree
  public void MoveWristToRawAngle(double angle){
    wristMaster.set(ControlMode.MotionMagic, angle);
    wristSlave.follow(wristMaster);
  }

  //Move wrist to a postion - in degree
  public void MoveWristToAngle(double angle){
    double targetAngle = angle;
    double setPoint;
    setPoint = (angle/360)*(countPerRev * (armRatio));
    System.out.println("Button Pressed. Moving to angle: " + targetAngle);
    wristMaster.set(ControlMode.MotionMagic, setPoint);
    wristSlave.follow(wristMaster);
  }
  
  
/**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking() {
        final double STEER_P = 0.03/2;              // how hard to turn toward the target
        final double DRIVE_P = 0.15;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 3.75;       // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.75;                   // Simple speed limit so we don't drive too fast
        final double STEER_I = 0.15;
        final double DRIVE_I = .75;
        final double xError;
        final double aError;
        double STEER_INTEGRAL = 0;
        double DRIVE_INTEGRAL = 0;

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        xError = tx;
        aError = DESIRED_TARGET_AREA - ta;
        STEER_INTEGRAL = STEER_INTEGRAL + (xError*0.02);
        DRIVE_INTEGRAL = DRIVE_INTEGRAL + (aError * 0.02);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = (tx * STEER_P) + (STEER_INTEGRAL * STEER_I);
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (aError * DRIVE_P) + (DRIVE_INTEGRAL * DRIVE_I);

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }
	
/** 
*
* @param angle target angle
* @param direction forward or backwards; 1.0 for forward, and -1.0 for backwards
*/
  public void PIDControl(int angle, double direction){
    double desiredAngle = angle;
    double max_speed = direction * 0.5;
    double turn_kP = 0.025;
    double turn_kI = 0.1;
    double integral = 0;
    double curentAngle = onboardGyro.getAngle();
    double error = desiredAngle - curentAngle;
    integral = integral + (error*0.02);
    double turnCmd = (error * turn_kP) + (turn_kI * integral) ;
    rightMasterMotor1.set(ControlMode.PercentOutput, max_speed + turnCmd);
    rightSlaveMotor2.follow(rightMasterMotor1);
    rightSlaveMotor3.follow(rightMasterMotor1);
    leftMasterMotor1.set(ControlMode.PercentOutput, max_speed - turnCmd);
    leftSlaveMotor2.follow(leftMasterMotor1);
    leftSlaveMotor3.follow(leftMasterMotor1);
  }
 
 
}
