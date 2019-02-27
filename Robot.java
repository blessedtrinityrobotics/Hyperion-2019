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
    VictorSPX elevLeftSlave     = new VictorSPX (11);
    //Right GB
    TalonSRX  elevRightMaster   = new TalonSRX  (9);
    VictorSPX elevRightSlave    = new VictorSPX (10);
  //WRIST
    TalonSRX  wristMaster       = new TalonSRX  (13);
    VictorSPX wristSlave        = new VictorSPX (12);
  //INTAKE
    VictorSPX intakeMotor       = new VictorSPX (14);
  //Joysticks
    private Joystick leftJoy;
    private Joystick rghtJoy;
    private XboxController operatorController;

  //Usually Variables
    //Encoder Counts per Revolution
    final private double countPerRev = 4096;
    //wheel Radius
    final private double wheelRadius = 3;
    //Wheel Circumference
    final private double wheelCircumference = 2* Math.PI * wheelRadius; //Circumference (in inches) (2*r*pi)
    final private double sprocketPitch =  1.79;
    double distance;
    double velocity; 
  //Gyro
  private ADXRS450_Gyro onboardGyro;
  //Limelight
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
    
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    //CONTROLLERS
      leftJoy     = new Joystick(0);
      rghtJoy     = new Joystick(1);  
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
      //Left
      wristMaster.setInverted(false); //Reverse direction
      wristSlave .setInverted(true); //Reverse direction
      wristMaster.setNeutralMode(NeutralMode.Brake);
      wristSlave .setNeutralMode(NeutralMode.Brake);
        //Encoder
        wristMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.kTimeoutMs);
        wristMaster.setSensorPhase(true); //Reverse direction
        //Config PID F Gains
        wristMaster.config_kP(Constants.kSlot_Wrist, Constants.kGains_Wrist.kP, Constants.kTimeoutMs);
        wristMaster.config_kI(Constants.kSlot_Wrist, Constants.kGains_Wrist.kI, Constants.kTimeoutMs);
        wristMaster.config_kD(Constants.kSlot_Wrist, Constants.kGains_Wrist.kD, Constants.kTimeoutMs);
        wristMaster.config_kF(Constants.kSlot_Wrist, Constants.kGains_Wrist.kF, Constants.kTimeoutMs);
        wristMaster.configMotionAcceleration(Constants.kWristAccel, Constants.kTimeoutMs);
        wristMaster.configMotionCruiseVelocity(Constants.kWristVel, Constants.kTimeoutMs);
    //END OF WRIST

    //INTAKE
      intakeMotor.setInverted(true);
      intakeMotor.setNeutralMode(NeutralMode.Brake);
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
    
      double rightEncoderPos = rightMasterMotor1.getSelectedSensorPosition(Constants.PID_PRIMARY);
      double rightEncoderVel = rightMasterMotor1.getSelectedSensorVelocity(Constants.PID_PRIMARY);
      //double leftEncoderPos = leftMasterMotor1.getSelectedSensorPosition();
    
      //double cimRPM = 4500;
      //double gearRatio = 6.23;
      //velocity = (cimRPM/600)*(countPerRev/gearRatio);

      distance = (rightEncoderPos * wheelCircumference)/countPerRev; 
      

      SmartDashboard.putNumber("Right Side Pos", distance);
      SmartDashboard.putNumber("Right Side Vel", rightEncoderVel);
      SmartDashboard.putNumber("Wrist Postion (encoder ticks): ", wristMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
      SmartDashboard.putNumber("Elevator Postion (encoder ticks): ", elevLeftMaster.getSelectedSensorPosition(Constants.PID_PRIMARY));

      //SmartDashboard.putNumber("Max Velocity", velocity);
      //System.out.println("Right Side Encoder Postion: " + distance);
    
    Update_Limelight_Tracking();
      double driveForwardPower;
      double turnPower;
    //Drive Train Controls
    if (leftJoy.getTrigger())
        {
          if (m_LimelightHasValidTarget)
          {
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
          }
          else
          {
            SmartDashboard.putString("Valid Target", "False");
          }
        }
        else
        {
          //Percent Control Mode
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
        }
      //move elev up 50 inches
      if(operatorJoy.getTrigger()){
        MoveElevToTarget(50.0);
      } else {
        elevLeftMaster.set(ControlMode.PercentOutput, operatorJoy.getY());
        elevRightMaster.set(ControlMode.PercentOutput, operatorJoy.getY());
        elevLeftSlave.follow(elevLeftMaster);
        elevRightSlave.follow(elevRightMaster);
      }
      
      //move wrist 45 degrees
      if(operatorJoy.getRawButtonPressed(2)){
        MoveWristToAngle(-45);
      }
   
    //Intake
      if(operatorController.getYButton()){
        intakeMotor.set(ControlMode.PercentOutput, 1.0);
      } else if(operatorController.getAButton()){
        intakeMotor.set(ControlMode.PercentOutput, -1.0);
      } else {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
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
    setPoint = (dist * countPerRev)/sprocketPitch;
    System.out.println("Button Pressed. Moving to: " + targetDist);
    elevRightMaster.set(ControlMode.MotionMagic, setPoint);
    elevRightSlave.follow(elevRightMaster);
    elevLeftMaster.set(ControlMode.MotionMagic, setPoint);
    elevLeftSlave.follow(elevLeftMaster);
  }

  //Move wrist to a postion - in degree
  public void MoveWristToAngle(double angle){
    double targetAngle = angle;
    double setPoint;
    setPoint = (360 * countPerRev)/(angle);
    System.out.println("Button Pressed. Moving to angle: " + targetAngle);
    wristMaster.set(ControlMode.MotionMagic, setPoint);
    wristSlave.follow(wristMaster);
  }
  
/**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.5;              // how hard to turn toward the target
        final double DRIVE_K = Constants.kGains_Distanc.kP;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }


}
