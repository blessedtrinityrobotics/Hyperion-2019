/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// A project by Ricardo Santamaria-Sarcos, Dario Zaccagnino, Christian Kolowich and David James

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.networktables.NetworkTableInstance;

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

     private XboxController operatorController;
    private Joystick leftJoy;
    private Joystick rghtJoy;
  
  private ADXRS450_Gyro onboardGyro;
  
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
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
      rightMasterMotor1.setInverted(true);
      rightSlaveMotor2.setInverted(true);
      rightSlaveMotor3.setInverted(true);
      rightMasterMotor1.configOpenloopRamp(1, 0);
      rightMasterMotor1.setNeutralMode(NeutralMode.Brake);
      rightSlaveMotor2.setNeutralMode(NeutralMode.Brake);
      rightSlaveMotor3.setNeutralMode(NeutralMode.Brake);
      
      //Sets up motor controller settings for left side
      leftMasterMotor1.setInverted(false);
      leftSlaveMotor2.setInverted(false);
      leftSlaveMotor3.setInverted(false);
      leftMasterMotor1.configOpenloopRamp(1, 0);
      leftMasterMotor1.setNeutralMode(NeutralMode.Brake);
      leftSlaveMotor2.setNeutralMode(NeutralMode.Brake);
      leftSlaveMotor3.setNeutralMode(NeutralMode.Brake);

    //END OF DRIVE TRAIN
    
    //ELEVATOR
      //Left
      elevLeftMaster.setInverted(true); //Reverse direction
      elevLeftSlave .setInverted(true); //Reverse direction
      elevLeftMaster.setNeutralMode(NeutralMode.Brake);
      elevLeftSlave .setNeutralMode(NeutralMode.Brake);
      elevLeftMaster.configOpenloopRamp(1.5);

    
      //Right
      elevRightMaster.setInverted(false); //Reverse direction
      elevRightSlave .setInverted(false); //Reverse direction
      elevRightMaster.setNeutralMode(NeutralMode.Brake);
      elevRightSlave .setNeutralMode(NeutralMode.Brake); 
      elevRightMaster.configOpenloopRamp(1.5);
     

    //END OF ELEVATOR

     
    //WRIST
      //Left
      wristMaster.setInverted(true); //Reverse direction
      wristSlave .setInverted(true); //Reverse direction
      wristMaster.setNeutralMode(NeutralMode.Brake);
      wristSlave .setNeutralMode(NeutralMode.Brake);

    //END OF WRIST
    
    
    //INTAKE
      intakeMotor.setInverted(true);
      intakeMotor.setNeutralMode(NeutralMode.Brake);
    //END OF INTAKE
    
/*
    //Camera
    UsbCamera camera = new UsbCamera("cam0", 0);
        camera.setBrightness(50);
    */

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
      //Elevator
        elevLeftMaster.set(ControlMode.PercentOutput, (operatorController.getY(Hand.kLeft)/2));
        elevRightMaster.set(ControlMode.PercentOutput, (operatorController.getY(Hand.kLeft)/2));
        elevLeftSlave.follow(elevLeftMaster);
        elevRightSlave.follow(elevRightMaster);

      //Wrist
        wristMaster.set(ControlMode.PercentOutput, operatorController.getY(Hand.kRight));
        wristSlave.follow(wristMaster);

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
