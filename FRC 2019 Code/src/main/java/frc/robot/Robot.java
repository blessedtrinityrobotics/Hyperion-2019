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
//Define Everything
  //Motor Controllers
  //Right Side Motors
  TalonSRX rightMasterMotor1 = new TalonSRX(1);
  VictorSPX rightSlaveMotor2 = new VictorSPX(2);
  VictorSPX rightSlaveMotor3 = new VictorSPX(3);
  //Left Side Motors
  TalonSRX leftMasterMotor1 = new TalonSRX(4);
  VictorSPX leftSlaveMotor2 = new VictorSPX(5);
  VictorSPX leftSlaveMotor3 = new VictorSPX(6);

  //Joysticks
  private Joystick leftJoy;
  private Joystick rghtJoy;

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

    //Sets up motor controller settings for right side
    rightMasterMotor1.setInverted(false);
    rightSlaveMotor2.setInverted(false);
    rightSlaveMotor3.setInverted(false);
    rightMasterMotor1.configOpenloopRamp(1, 0);
    rightMasterMotor1.setNeutralMode(NeutralMode.Brake);
    rightSlaveMotor2.setNeutralMode(NeutralMode.Brake);
    rightSlaveMotor3.setNeutralMode(NeutralMode.Brake);
    //Encoder Right Side
    rightMasterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    //Sets up motor controller settings for left side
    leftMasterMotor1.setInverted(true);
    leftSlaveMotor2.setInverted(true);
    leftSlaveMotor3.setInverted(true);
    leftMasterMotor1.configOpenloopRamp(1, 0);
    leftMasterMotor1.setNeutralMode(NeutralMode.Brake);
    leftSlaveMotor2.setNeutralMode(NeutralMode.Brake);
    leftSlaveMotor3.setNeutralMode(NeutralMode.Brake);
    //Encoder Left Side
    leftMasterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    //Reverse Direction of Encoder
    leftMasterMotor1.setSensorPhase(true);
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
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
