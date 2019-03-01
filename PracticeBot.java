/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.hal.RelayJNI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
  

  VictorSP rightFirst = new VictorSP(0);
  VictorSP rightSecond = new VictorSP(1);
  VictorSP leftFirst =  new VictorSP(2);
  VictorSP leftSecond = new VictorSP(3); 

  Joystick JRight = new Joystick(0);
  Joystick JLeft = new Joystick(1);
  
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
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Gyro
    onboardGyro = new  ADXRS450_Gyro();
    onboardGyro.calibrate();//Calibrates the gyro
    onboardGyro.reset();//Sets gyro to 0 degrees

    //set directions
    rightFirst.setInverted(true);
    rightSecond.setInverted(true);
    leftFirst.setInverted(false);
    leftSecond.setInverted(false);
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during auton
   * omous.
   */
  @Override
  public void autonomousPeriodic() {
    int counter = 0;
    switch (m_autoSelected) {
      case kCustomAuto:
        
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
    SmartDashboard.putNumber("Gyro:", onboardGyro.getAngle());
    Update_Limelight_Tracking();
      double driveForwardPower;
      double turnPower;
    //Drive Train Controls
    if (JRight.getTrigger())
        {
          if (m_LimelightHasValidTarget)
          {
            //Limelight vision procesing control
            SmartDashboard.putString("Valid Target", "True");
            double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
            SmartDashboard.putNumber("TX", tx);
            SmartDashboard.putNumber("TA", ta);
            driveForwardPower = m_LimelightDriveCommand;
            turnPower = m_LimelightSteerCommand;
            rightFirst.set(driveForwardPower-turnPower);
            rightSecond.set(driveForwardPower-turnPower);
            leftFirst.set(driveForwardPower+turnPower);
            leftSecond.set(driveForwardPower+turnPower);
          }
          else
          {
            SmartDashboard.putString("Valid Target", "False");
          }
        }
        else
        {
          rightFirst.set(-JRight.getY() - JRight.getX()/2);
          rightSecond.set(-JRight.getY() - JRight.getX()/2);
          leftFirst.set(-JRight.getY() + JRight.getX()/2);
          leftSecond.set(-JRight.getY() + JRight.getX()/2);
        }
    

    if(JRight.getRawButton(2)){
      onboardGyro.reset();
    }

    if(JRight.getRawButton(5)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

    } 
    if(JRight.getRawButton(4)){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    }
    if(JRight.getRawButton(3)){
      double desiredAngle = 0;
      double max_speed = 0.5;
      double turn_kP = 0.015;
      double curentAngle = onboardGyro.getAngle();
      double turnCmd = (desiredAngle - curentAngle) * turn_kP;
      SmartDashboard.putNumber("Error:", desiredAngle-curentAngle);
      rightFirst.set(max_speed + turnCmd);
      rightSecond.set(max_speed + turnCmd);
      leftFirst.set(max_speed - turnCmd);
      leftSecond.set(max_speed - turnCmd);
    }

    if(JLeft.getTrigger()){
      double desiredAngle = 90;
      double max_speed = 0.25;
      double turn_kP = 0.005;
      double curentAngle = onboardGyro.getAngle();
      double turnCmd = (desiredAngle - curentAngle) * turn_kP;
      SmartDashboard.putNumber("Error:", desiredAngle-curentAngle);
      rightFirst.set(max_speed + turnCmd);
      rightSecond.set(max_speed + turnCmd);
      leftFirst.set(max_speed - turnCmd);
      leftSecond.set(max_speed - turnCmd);
    }
    
    if(JRight.getRawButton(3)){ //Drive Straight Forward
        PIDControl(0, 1.0);
    } else if(JRight.getRawButton(2)){ //Drive Straight Backwards
        PIDControl(0, -1.0);
    } else if(JRight.getRawButton(4)){ //Turn Left 90 and continue driving straight
        PIDControl(90, 1.0);
    } else if(JRight.getRawButton(5)){ //Turn Right 90 and continue driving straight
        PIDControl(-90, 1.0);
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
        final double STEER_K = 0.02;              // how hard to turn toward the target
        final double DRIVE_K = 0.1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 3.5;       // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.75;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
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
  
  /** 
*
* @param angle target angle
* @param direction forward or backwards; 1.0 for forward, and -1.0 for backwards
*/
  public void PIDControl(int angle, double direction){
    double desiredAngle = angle;
    double max_speed = direction * 0.5;
    double turn_kP = 0.015;
    double turn_kI = 0;
    double integral = 0;
    double curentAngle = onboardGyro.getAngle();
    double error = desiredAngle - curentAngle;
    integral = integral + (error*0.02);
    double turnCmd = (error * turn_kP) + (turn_kI * integral) ;
    SmartDashboard.putNumber("Error:", error);
    rightFirst.set(max_speed + turnCmd);
    rightSecond.set(max_speed + turnCmd);
    leftFirst.set(max_speed - turnCmd);
    leftSecond.set(max_speed - turnCmd);
  }
}
