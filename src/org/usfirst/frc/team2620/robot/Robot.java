package org.usfirst.frc.team2620.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;

public class Robot extends TimedRobot{
	
	WPI_TalonSRX driveRight = new WPI_TalonSRX(5);
	WPI_TalonSRX driveLeft = new WPI_TalonSRX(1);
	
	/*WPI_TalonSRX driveRight = new WPI_TalonSRX(1);
	WPI_TalonSRX driveLeft = new WPI_TalonSRX(5);*/

	WPI_TalonSRX stage2Right = new WPI_TalonSRX(2);
	WPI_TalonSRX stage2Left = new WPI_TalonSRX(6);
	WPI_TalonSRX carriageMotor = new WPI_TalonSRX(7);
	//WPI_TalonSRX pickupRight = new WPI_TalonSRX(3);
	//WPI_TalonSRX pickupLeft = new WPI_TalonSRX(4);
	WPI_TalonSRX tiltMotor = new WPI_TalonSRX(8);
	
	//fix b4 comp
	PWMSpeedController pickupLeft = new Spark(3);
	PWMSpeedController pickupRight = new Spark(4);
	
	Servo RclimbLock = new Servo(0);
	Servo LclimbLock = new Servo(1);
	Servo arms = new Servo(2);
	
	Encoder driveLeftEncoder = new Encoder(3, 5, false, Encoder.EncodingType.k4X);
	Encoder driveRightEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);	
	
	double encoderD = 0;
	
	Ultrasonic frontDistance = new Ultrasonic(8, 9);
	//ADXRS450_Gyro gyro;
	
	DigitalInput stage2TopStop = new DigitalInput(2);
	DigitalInput stage2BottomStop = new DigitalInput(4);
	DigitalInput carriageTopStop = new DigitalInput(1);
	DigitalInput carriageBottomStop = new DigitalInput(0);
	
	//DifferentialDrive drive;
	
	Joystick left = new Joystick(0);
	Joystick right = new Joystick(1);
	
	double pickupSpeed = 1.0;
	double stage2Speed = 1.0;
	double carriageSpeed = 1.0;
	private SendableChooser<Integer> driveChooser;
	private boolean Locked;
	private SendableChooser<Integer> enableSwitchlocks;
	
	int difference;
	
	// Auton Vars
	private SendableChooser<Integer> autonChooser;
	private int autonMode;
	private String autonGameData;
	private double autonDriveSpeed = 0.2; // In % Power
	private int autonStraightOnlyDistance = 5 * 12;
	private boolean auton_atTarget = false;
	private boolean auton_centered = false;
	private boolean auton_atHeight = false;
	private String auton_sideLastSeen; // Side relevant to the robot. L from gamedata would mean its on the right by default
	private boolean autonLifted = false;
	private Timer autonTimer;

	boolean stop = false; 
	boolean rightStop = false;
	boolean leftStop = false;
	
	private Timer carriageBumpTimer;
	private boolean carriageBumpRun = false;
	
	DifferentialDrive diffDrive = new DifferentialDrive(driveLeft, driveRight);
	
	public Robot(){
		
		CameraServer.getInstance().startAutomaticCapture();
	}
	
	public void robotInit()
	{
		// TODO: ROBOT NEEDS ULTRASONIC AND CAMERA AND LIMIT SWITCHES
		// CameraServer.getInstance().startAutomaticCapture();
		
		CameraServer.getInstance().startAutomaticCapture();
		
		carriageBumpTimer = new Timer();

		arms.set(120);
		autonTimer = new Timer();
		
		//gyro = new ADXRS450_Gyro();
		//gyro.calibrate();
		
		driveRight.setInverted(true);
		pickupLeft.setInverted(true);
		stage2Left.setInverted(true);

		/*pickupLeft.setNeutralMode(NeutralMode.Brake);
		pickupRight.setNeutralMode(NeutralMode.Brake);*/
		
		// Setup Encoders, https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
		driveLeftEncoder.setMaxPeriod(.1);
		driveLeftEncoder.setMinRate(10);
		driveLeftEncoder.setDistancePerPulse(6.0 / 360.0); // Distance = circumference * pulses / pulses_per_revolution, circumference = 6, pulsers_per_revolution = 360?
		driveLeftEncoder.setReverseDirection(false);
		driveLeftEncoder.setSamplesToAverage(7);

		driveRightEncoder.setMaxPeriod(.1);
		driveRightEncoder.setMinRate(10);
		driveRightEncoder.setDistancePerPulse(6.0 / 360.0); // Distance = circumference * pulses / pulses_per_revolution, circumference = 6, pulsers_per_revolution = 360?
		driveRightEncoder.setReverseDirection(true);
		driveRightEncoder.setSamplesToAverage(7);

		// Setup Auton Chooseable
		autonChooser = new SendableChooser<Integer>();
		autonChooser.addDefault("ROBOT STRAIGHT ONLY", 1);
		autonChooser.addObject("TARGET SWITCH - ROBOT is LEFT", 2);
		autonChooser.addObject("TARGET SWITCH - ROBOT is RIGHT", 3);
		autonChooser.addObject("TARGET SCALE -- ROBOT is LEFT", 4);
		autonChooser.addObject("TARGET SCALE -- ROBOT is RIGHT", 5);
		SmartDashboard.putData("Auton Mode", autonChooser);
		

		driveChooser = new SendableChooser<Integer>();
		driveChooser.addDefault("Jacob", 1);
		driveChooser.addObject("Chloe", 2);
		SmartDashboard.putData("Driver", driveChooser);
		
		enableSwitchlocks = new SendableChooser<Integer>();
		enableSwitchlocks.addDefault("Carriage Switch Locks Enabled", 1);
		enableSwitchlocks.addObject("Carriage Switch Locks Disabled", 2);
		SmartDashboard.putData("Carriage Switch Locks", enableSwitchlocks);
		
		Locked = false;
		updateSmartDashboard();
	}
	
	public void updateSmartDashboard() 
	{
		SmartDashboard.putBoolean("Carriage Top Stop", carriageTopStop.get());
		SmartDashboard.putBoolean("Carriage Bottom Stop", carriageBottomStop.get());
		SmartDashboard.putBoolean("Stage 2 Top Stop", stage2TopStop.get());
		SmartDashboard.putBoolean("Stage 2 Bottom Stop", stage2BottomStop.get());
		SmartDashboard.putBoolean("Climb Lock Enabled", Locked);
		SmartDashboard.putNumber("Right", driveRightEncoder.getRaw());
		SmartDashboard.putNumber("Left", driveLeftEncoder.getRaw());
	}

	public void drive(double left, double right)
	{
		// Powers the drives motors, guarantees positive is forward for both left & right
		if(!stage2BottomStop.get()) {
			left *= 0.5;
			right *= 0.5;
		}

		driveRight.set(ControlMode.PercentOutput, right);
		driveLeft.set(ControlMode.PercentOutput, left);
	}

	public void pickup(double speed)
	{
		// Runs both left and right pickup motors at the same time
		pickupRight.set(speed);
		pickupLeft.set(speed);
	}
	
	public void lift(double speed) 
	{
		
		if(stage2TopStop.get() && speed > 0) {
			speed = 0.0;
		}
		
		if(stage2BottomStop.get() && speed < 0) {
			speed = 0.0;
		}

		stage2Right.set(speed);
		stage2Left.set(speed);
	}
	
	public void carriage(double speed) 
	{	
		if(enableSwitchlocks.getSelected() == 1) {
			if(carriageTopStop.get() && speed > 0) {
				speed = 0.0;
			}
			
			if(carriageBottomStop.get() && speed < 0) {
				speed = 0.0;
				
				if(!carriageBumpRun) {
					carriageBumpTimer.reset();
					carriageBumpTimer.start();
					carriageBumpRun = true;
				}
			}
		}
		
		if(carriageBumpRun) {
			if(carriageBumpTimer.get() < 0.15) {
				speed = 0.4;
			} else {
				speed = 0.0;
				carriageBumpRun = false;
				carriageBumpTimer.stop();
			}
		}
		
		carriageMotor.set(speed);
	}
	
	public void encMove(double speed, int distance){
		driveLeftEncoder.reset();
		driveRightEncoder.reset();
		
		while(stop = false){
			double newSpeed = speed;
				difference = driveRightEncoder.getRaw() - driveLeftEncoder.getRaw();
				if(difference > 10){
					newSpeed = (newSpeed - .10);
					driveRight.set(newSpeed);
				}
				else{
					driveRight.set(speed);
					newSpeed = speed;
				}
				if(difference < -10){
					newSpeed = (newSpeed - .10);
					driveLeft.set(newSpeed);
				}
				else{
					driveLeft.set(speed);
					newSpeed = speed;
				}
			if(driveRightEncoder.getRaw() > distance){
				rightStop = true;
				driveRight.set(0);
				}
			if(driveLeftEncoder.getRaw() > distance){
				leftStop = true;
				driveLeft.set(0);
			}
			if(leftStop == true && rightStop == true){
				stop = true;
			}
		
		
		}
	}
	public void encMove(double lSpeed, int lDistance, double rSpeed, int rDistance) {
		driveLeftEncoder.reset();
		driveRightEncoder.reset();
		boolean stop = false;
		boolean leftStop = false;
		boolean rightStop = false;
		while(stop == false){
			if(driveRightEncoder.getRaw() < rDistance){
				driveRight.set(rSpeed);
			}
			else{
				driveRight.set(0);
				rightStop = true;
			}
			if(driveLeftEncoder.getRaw() < lDistance){
				driveLeft.set(lSpeed);
			}
			else{
				driveLeft.set(0);
				leftStop = true;
			}
			if(rightStop == true && leftStop == true){
				stop = true;
			}
		}
		
	}

	public void autonomousInit()
	{	
		
		autonLifted = false;
		autonTimer.reset();
		autonTimer.start();
		
		//gyro.reset();
		
		auton_atTarget = false;
		auton_centered = false;
		auton_atHeight = false;

		autonGameData = DriverStation.getInstance().getGameSpecificMessage();
		autonMode = (int) autonChooser.getSelected();
		frontDistance.setAutomaticMode(true);
		driveLeftEncoder.reset();
		driveRightEncoder.reset();
		driveLeft.setInverted(true);
		driveRight.setInverted(false);
		
		
	}

	public void autonomousPeriodic()
	{	
		
		updateSmartDashboard();
		double driveForwardTime = 1.5;
		
		
		autonLifted = true;
		if(!autonLifted) {
			carriage(0.4);
			Timer.delay(0.3);
			carriage(0.0);
			autonLifted = true;
		} else {
			switch(autonMode) {
				case 1:
					// System.out.println(autonTimer.get());
					if(autonTimer.get() >= driveForwardTime) {
						// 2 - Stop driving
						drive(0.0, 0.0);
					} else {
						// 1 - Start
						drive(0.7, 0.7);
					}
					break;
				case 2: // Switch & Robot is on Left
					
				case 3: // Switch & Robot is on Right
					
					
					if (autonMode == 2 && autonGameData.charAt(0) == 'L') { //if Robot and switch are on Left
						
							if(stop == false){
								encMove(.5, 9500);
								Timer.delay(1);
								encMove(.25, 7000, .25, 0);
								Timer.delay(1);
								encMove(.25, 3000);
								Timer.delay(1);
								stop = true;
							}
						
					}
					if (autonMode == 2 && autonGameData.charAt(0) == 'R'){//if robot left and switch is on Right
						if(stop == false){	
							encMove(.5, 11000);
							Timer.delay(1);
							encMove(.25, 3100, .25, 0);
							Timer.delay(1);
							encMove(.5, 500);
							Timer.delay(1);
							encMove(.25, 3100, .25, 0);
							Timer.delay(1);
							stop = true;
						}
					}
					
					if (autonMode == 3 && autonGameData.charAt(0) == 'R') { // if Robot and switch are on Right
						encMove(.5, 9500);
					}
					if(autonMode == 3 && autonGameData.charAt(0) == 'L'){//if robot right and switch are Left
						driveRight.set(-1);Timer.delay(0.1);driveLeft.set(0);
					}
					
					break;
				case 4: // Scale & Robot is on Left
					auton_sideLastSeen = "R";
					break;
				case 5: // Scale & Robot is on Right
					auton_sideLastSeen = "L";
					// auton_placeInScale();
					break;
				default:
					// auton_driveStraightDistance(autonStraightOnlyDistance);
					break;
			}
		}
	}

	public void teleopInit() {
		Locked = false;
		//gyro.reset();
		driveLeftEncoder.reset();
		driveRightEncoder.reset();
	}
	public void teleopPeriodic(){
		updateSmartDashboard();
		
		if(Locked == false){
		RclimbLock.setAngle(180);
		LclimbLock.setAngle(180);
		}
		
		

		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////
		// Change true for tank and false for arcade ///////////////////////
		boolean Jacob = driveChooser.getSelected() == 1;
		
		
		if(Jacob == false) {
			if(stage2BottomStop.get()){
			diffDrive.arcadeDrive(right.getRawAxis(0)  * -1, right.getRawAxis(1));
			}
			else{
				diffDrive.arcadeDrive(right.getRawAxis(0) * .76  * -1, right.getRawAxis(1) * .76);
			}
		}
		
		
		if(Jacob == true) {
			if(stage2BottomStop.get()){
			driveRight.set(ControlMode.PercentOutput, right.getRawAxis(1));
			driveLeft.set(ControlMode.PercentOutput, left.getRawAxis(1));
			}
			else{
				driveRight.set(ControlMode.PercentOutput, right.getRawAxis(1) * .76);
				driveLeft.set(ControlMode.PercentOutput, left.getRawAxis(1)* .76);
			}
		}
			
		
		boolean lTrigger = left.getRawButton(1);
		boolean lFaceB = left.getRawButton(2);
		boolean lFaceL = left.getRawButton(3);
		boolean lFaceR = left.getRawButton(4);
		boolean lPLTLeft = left.getRawButton(11);
		boolean lPLTCenter = left.getRawButton(12);
		boolean lPLTRight = left.getRawButton(13);
		boolean lPLBLeft = left.getRawButton(16);
		boolean lPLBCenter = left.getRawButton(15);
		boolean lPLBRight = left.getRawButton(14);
		boolean lPRTLeft = left.getRawButton(7);
		boolean lPRTCenter = left.getRawButton(6);
		boolean lPRTRight = left.getRawButton(5);
		boolean lPRBLeft = left.getRawButton(8);
		boolean lPRBCenter = left.getRawButton(9);
		boolean lPRBRight = left.getRawButton(10);
		int lPOV = left.getPOV();  // Top Directional Thumb on joystick
		
		boolean rTrigger = right.getRawButton(1);
		boolean rFaceB = right.getRawButton(2);
		boolean rFaceL = right.getRawButton(3);
		boolean rFaceR = right.getRawButton(4);
		boolean rPLTLeft = right.getRawButton(5);
		boolean rPLTCenter = right.getRawButton(12);
		boolean rPLTRight = right.getRawButton(13);
		boolean rPLBLeft = right.getRawButton(16);
		boolean rPLBCenter = right.getRawButton(15);
		boolean rPLBRight = right.getRawButton(14);
		boolean rPRTLeft = right.getRawButton(7);
		boolean rPRTCenter = right.getRawButton(6);
		//boolean rPRTRight = right.getRawButton(5);
		boolean rPRBLeft = right.getRawButton(8);
		boolean rPRBCenter = right.getRawButton(9);
		boolean rPRBRight = right.getRawButton(10);
		int rPOV = right.getPOV();  // Top Directional Thumb on joystick
		
		//lock
		
		
		if(Jacob == true) {
			if(left.getRawButton(3)){
					RclimbLock.setAngle(0);
					LclimbLock.setAngle(45);
					Locked = true;
				}
		}
		
		if(Jacob == false) {
			if(rPLTLeft) {
				Locked = true;
				RclimbLock.setAngle(0);
				LclimbLock.setAngle(45);
			}
		}
		
		
		
		//System.out.println(left.getRawAxis(3));
		
		// Pickup Logic
		if(Jacob == true) {
			if(rTrigger) {
				pickup(pickupSpeed);
				
			}
			else if(lTrigger) {
				pickup(pickupSpeed * -1);
			}
			else {
				

				// set to 0.1 for operation
				pickup(0.15);
			}

		}
		if(Jacob == false) {
			if(rTrigger) {
				pickup(pickupSpeed);
			
			}
			else if(rFaceB) {
				pickup(pickupSpeed * -1);
			}
			else {
				// set to 0.1 for operation
				pickup(0.15);
			}
		}

		
		
		// Stage 2 Logic
		/*if(Jacob == true) {
			if(lPOV == 0 || lPOV == 45 || lPOV == 315) {
				lift(stage2Speed);
			}
			else if(lPOV == 180 || lPOV == 285 || lPOV == 135) {
				lift(stage2Speed * -1);
			}
			else {
			lift(0.07);
			}
		}*/

		// Carriage
		if(rPOV == 0 || rPOV == 45 || rPOV == 315) {
			if(!stage2TopStop.get()){
				stage2Left.set(1.0);
			}
			else{
				stage2Left.set(0.0);
			}
			if(!carriageTopStop.get()){
				carriage(1.0);
			}
			else{
				carriage(0.0);
			}
		}
		else{
			stage2Left.set(0);
			carriage(0.1);
		}
		if(rPOV == 180 || rPOV == 285 || rPOV == 135){
			if(!stage2BottomStop.get()){
				stage2Left.set(-1);
			}
			else{
				stage2Left.set(0);
				carriage(-1);
			}
		}
		
		
		////////////////////////////////////////////
/*		if(Jacob == true) {
			if(rPOV == 0 || rPOV == 45 || rPOV == 315) {
				carriage(carriageSpeed);
			} else {			
				if(rPOV == 180 || rPOV == 285 || rPOV == 135) {	
					if(!stage2BottomStop.get()) {
						stage2Left.set(-1);
					}

					if(stage2BottomStop.get()) {
						carriage(carriageSpeed * -1);
					}
				} else {
					carriage(0.07);
				}
				
			}
		}*/
	
		// OTHER BOT PICKUP ARMS
		
		if(right.getRawButton(16)){
			arms.setAngle(40);
			Timer.delay(6);
			arms.set(120);
		}
		
		//ARM TILT LOGIC
		if(rFaceR){
			tiltMotor.set(.75);
		}
		else{
			if(rFaceL){
				tiltMotor.set(-.75);
			}
			else{
				tiltMotor.set(0);
			}
		}
	}

		
	

	public void testPeriodic(){
		
		 
		/*boolean lTrigger = left.getRawButton(1);
		
		if(lTrigger) {
			carriage(0.4);
			Timer.delay(0.3);
			carriage(0.0);
		}*/
		
		
		
		
		/*boolean lTrigger = left.getRawButton(1);
		boolean rTrigger = right.getRawButton(1);
		
		if(lTrigger) {
			carriage(1.0);
		} else {
			if(rTrigger) {
				carriage(-1.0);
			} else {
				carriage(0.1);
			}
		}*/
		
//		System.out.println("stage2TopStop");
//		System.out.println(stage2TopStop.get());
//		System.out.println(stage2TopStop.get());
//		System.out.println(carriageTopStop.get());
//		System.out.println("carriageTopStop");
//		System.out.println(carriageTopStop.get());
//		System.out.println(carriageBottomStop.get());
//		System.out.println(carriageBottomStop.get());
	}
}
