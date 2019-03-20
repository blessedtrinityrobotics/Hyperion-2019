/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class Constants {
	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	//Elev peek sensor velocity 
	public final static int kElevSensorVelocity  = 3410;
	
	//Acceleration of Elevator in sensorUnitsPer100msPerSec
	public final static int kElevAccel = kElevVel/2 ;
	
	//speed of Elevator in sensorUnitsPer100ms
	public static int kElevVel = kElevSensorVelocity/2 ;
	
	
	//Wrist peek sensor velocity
	public final static int kWristSensorVel = 399;
	
	//Acceleration of wrist in sensorUnitsPer100msPerSec
	public final static int kWristAccel = kWristVel/2;
	
	//Speed of Elevator in sensorUnitsPer100ms
	public final static int kWristVel = kWristSensorVel/2;


	//Climb peek sensor velocity
	public final static int kClimbSensorVelocity = 1000;

	//Acceleration of climb in sensorUnitsPer100msPerSec
	public final static int kClimbAccel = kClimbVel/2;

	//speed of climb in sensorUnitsPer100ms
	public static int kClimbVel = kClimbSensorVelocity/2;
	/**
	 * Elevator Postion Targets
	 * Top
	 * Mid
	 * Low
	 * Bot
	 */
	public final static int elevTopGoal = 19000;
	public final static int elevMidGoal = 10000;
	public final static int elevLowGoal = 5000;
	public final static int elevBotGoal = 0;

	/**
	 * Wrist Targets
	 * Straight (Up)
	 * Down (Intake)
	 * Hatch 
	 */
	public static final int wristStraight = 0;
	public static final int wristDown     = 3000;
	public static final int wristHatch    = 2500

	/** 
	 * Climb Targets
	 * Extend
	 * Retract
	 */
	public static final int climbExtend  = 1000;
	public static final int climbRetract = 0;
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    	       kP   kI   kD   kF                          Iz   PeakOut */
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,                       100,  0.50 );
	public final static Gains kGains_Elev    = new Gains( 1.5, 0.0,  0.0, 1023/kElevSensorVelocity,  500,  0.50 );
	public final static Gains kGains_Wrist   = new Gains( 0.0, 0.0,  0.0, 1023/kWristSensorVel,      500,  0.50 );
	public final static Gains kGains_Climb   = new Gains( 0.0, 0.0,  0.0, 1023/kClimbSensorVelocity, 500,  0.50 );

	
	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = 0;
	public final static int kSlot_Elev = 1;
	public final static int kSlot_Wrist = 2;
	public final static int kSlot_Climb = 3;
}
