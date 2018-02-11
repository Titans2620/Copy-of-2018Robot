package org.usfirst.frc.team2620.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;

public class Robot extends TimedRobot {
	
	WPI_TalonSRX driveRight = new WPI_TalonSRX(5);
	WPI_TalonSRX driveLeft = new WPI_TalonSRX(1);

	WPI_TalonSRX stage2Right = new WPI_TalonSRX(2);
	WPI_TalonSRX stage2Left= new WPI_TalonSRX(2);
	WPI_TalonSRX carriageMotor = new WPI_TalonSRX(3);
	WPI_TalonSRX pickupRight = new WPI_TalonSRX(6);
	WPI_TalonSRX pickupLeft = new WPI_TalonSRX(7);
	
	Servo climbLock = new Servo(0);
	
	Encoder driveLeftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder driveRightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);	
	
	Ultrasonic frontDistance = new Ultrasonic(8, 9);
	ADXRS450_Gyro gyro;
	
	DigitalInput stage2TopStop = new DigitalInput(4);
	DigitalInput stage2BottomStop = new DigitalInput(5);
	DigitalInput carriageTopStop = new DigitalInput(6);
	DigitalInput carriageBottomStop = new DigitalInput(7);	
	
	Joystick left = new Joystick(0);
	Joystick right = new Joystick(1);
	
	double pickupSpeed = 1.0;
	double stage2Speed = 1.0;
	double carriageSpeed = 1.0;
	
	// Auton Vars
	private SendableChooser<Integer> autonChooser;
	private int autonMode;
	private String autonGameData;
	private double autonDriveSpeed = 0.2; // In % Power
	private int autonStraightOnlyDistance = 5 * 12;
	private boolean auton_atTarget = false;
	private boolean auton_centered = false;
	private boolean auton_atHeight = false;
	private String auton_sideLastSeen; // Side relevent to the robot. L from gamedata would mean its on the right by default
	
	public void robotInit()
	{
		// TODO: ROBOT NEEDS ULTRASONIC AND CAMERA AND LIMIT SWITCHES
		CameraServer.getInstance().startAutomaticCapture();

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();

		driveLeft.setInverted(true);
		pickupLeft.setInverted(true);
		stage2Left.setInverted(true);

		// Setup Encoders, https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599717-encoders-measuring-rotation-of-a-wheel-or-other-shaft
		driveLeftEncoder.setMaxPeriod(.1);
		driveLeftEncoder.setMinRate(10);
		driveLeftEncoder.setDistancePerPulse(6.0 / 360.0); // Distance = circumference * pulses / pulses_per_revolution, circumference = 6, pulsers_per_revolution = 360?
		driveLeftEncoder.setReverseDirection(false);
		driveLeftEncoder.setSamplesToAverage(7);

		driveRightEncoder.setMaxPeriod(.1);
		driveRightEncoder.setMinRate(10);
		driveRightEncoder.setDistancePerPulse(6.0 / 360.0); // Distance = circumference * pulses / pulses_per_revolution, circumference = 6, pulsers_per_revolution = 360?
		driveRightEncoder.setReverseDirection(false);
		driveRightEncoder.setSamplesToAverage(7);

		// Setup Auton Chooseable
		autonChooser = new SendableChooser<Integer>();
		autonChooser.addDefault("ROBOT STRAIGHT ONLY", 1);
		autonChooser.addObject("TARGET SWITCH - ROBOT is LEFT", 2);
		autonChooser.addObject("TARGET SWITCH - ROBOT is RIGHT", 3);
		autonChooser.addObject("TARGET SCALE -- ROBOT is LEFT", 4);
		autonChooser.addObject("TARGET SCALE -- ROBOT is RIGHT", 5);
		SmartDashboard.putData("Auton Mode", autonChooser);
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
		if(stage2BottomStop.get() && speed > 0) {
			speed = 0.0;
		}
		
		if(stage2TopStop.get() && speed < 0) {
			speed = 0.0;
		}

		stage2Right.set(speed);
		stage2Left.set(speed);
	}
	
	public void carriage(double speed) 
	{
		if(carriageTopStop.get() && speed > 0) {
			speed = 0.0;
		}
		
		if(carriageBottomStop.get() && speed < 0) {
			speed = 0.0;
		}

		carriageMotor.set(speed);
	}

	public void autonomousInit()
	{
		gyro.reset();
		
		auton_atTarget = false;
		auton_centered = false;
		auton_atHeight = false;

		autonGameData = DriverStation.getInstance().getGameSpecificMessage();
		autonMode = (int) autonChooser.getSelected();
		frontDistance.setAutomaticMode(true);
		driveLeftEncoder.reset();
		driveRightEncoder.reset();
	}

	private void auton_driveStraightOnly()
	{
		double speed = autonDriveSpeed;

		// Drive straight by adjusting the power to the motors by the % difference in rate in encoders
		double leftRate = driveLeftEncoder.getRate();
		double rightRate = driveRightEncoder.getRate();

		double leftRateAdjustment = (leftRate - rightRate) / (leftRate + rightRate);
		double rightRateAdjustment = (rightRate - leftRate) / (leftRate + rightRate);

		drive(speed * leftRateAdjustment, speed * rightRateAdjustment);
	}

	private boolean auton_driveStraightDistance(int inches) 
	{
		// Drives straight until so many inches are reached by the left encoder
		double leftDistance = driveLeftEncoder.getDistance();

		if(leftDistance < inches) {
			auton_driveStraightOnly();
		} else {
			drive(0.0, 0.0);
			return true;
		}
		return false;
	}

	private void auton_placeInSwitch() 
	{
		boolean shouldPlaceLeft = (autonGameData.charAt(0) == 'L' && autonMode == 2);
		boolean shouldPlaceRight = (autonGameData.charAt(0) == 'R' && autonMode == 3);

		if(!shouldPlaceLeft && !shouldPlaceRight) {
			// If our switch isnt on robots side, just drive past black line
			auton_driveStraightDistance(autonStraightOnlyDistance);
		} else {
			if(Timer.getMatchTime() < 1.0) {
				drive(autonDriveSpeed, autonDriveSpeed);
			} else {
	
				if(shouldPlaceLeft || shouldPlaceRight) 
				{
					if(!auton_centered) {
						// Turn left or right to center the robot on the target
	
						// Resolution is 640x480 I think?
	
						NetworkTableInstance instance = NetworkTableInstance.getDefault();
			  			NetworkTable rootTable = instance.getTable("GRIP");
						double centerX = rootTable.getSubTable("contours").getEntry("centerX").getDouble(640.0);
						double centerPercent = centerX / 640.0;
						double centerErrorPercent = 10.0;
	
						if(centerPercent > (50.0 - centerErrorPercent)) {
							auton_sideLastSeen = "R";
						} else if(centerPercent < (50.0 - centerErrorPercent)) {
							auton_sideLastSeen = "L";
						} else {
							auton_sideLastSeen = "C";
							auton_centered = true;
						}
	
						double leftDirection = 0.0;
						double rightDirection = 0.0;
	
						if(auton_sideLastSeen == "R") {
							// Turn robot left to get to the center
							leftDirection = -1.0;
							rightDirection = 1.0;
						} else if (auton_sideLastSeen == "L") {
							// Turn robot right to get to the center
							leftDirection = 1.0;
							rightDirection = -1.0;
						}
	
						drive(leftDirection * 0.2, rightDirection * -0.2);
					} else if (!auton_atTarget) {
						// Drive forward until we are very close to the target
						if (frontDistance.getRangeInches() < 6) { 
							// Within 6 inches of something
							auton_atTarget = true;
							drive(0.0, 0.0);
						} else {
							auton_driveStraightOnly();
						}
					} else {
						// Spit out the crate
						pickup(pickupSpeed * -1);
					}
				} else {
					auton_driveStraightOnly();
				}
			}
		}
	}

	private void auton_placeInScale()
	{		
		boolean shouldPlaceLeft = (autonGameData.charAt(1) == 'R' && autonMode == 4);
		boolean shouldPlaceRight = (autonGameData.charAt(1) == 'L' && autonMode == 5);
		
		if(!shouldPlaceLeft && !shouldPlaceRight) {
			// If our scale isnt on robots side, just drive past black line
			auton_driveStraightDistance(autonStraightOnlyDistance);
		} else {
			// Scale is scoreable from our position

			// TODO: REMOVE THE TWO HARD BOOL SETS BELOW
			auton_centered = true; // Skip this step
			auton_atHeight = true; // Skip this step
			
			if(!auton_centered) {
				// Drive forward directly next too scale
				auton_centered = auton_driveStraightDistance(10 * 12); // Drives forward 10 feet
			} else if (!auton_atHeight) {
				// Move to required height to place on scale
	
				lift(stage2Speed);
				carriage(carriageSpeed);
	
				if(carriageTopStop.get() && stage2TopStop.get()) {
					auton_atHeight = true;
					gyro.reset();
				}
			} else if(!auton_atTarget) {
				// Turn 90 degrees to face target
				
				System.out.println(gyro.getAngle());
	
				if(shouldPlaceLeft) {
					// Turn Right 90 degrees
					drive(autonDriveSpeed, autonDriveSpeed * -1.0);
					if(gyro.getAngle() > 88) {
						drive(0.0, 0.0);
						auton_atTarget = true;
					}
				} else if(shouldPlaceRight) {
					// Turn Left 90 degrees
					drive(autonDriveSpeed * -1.0, autonDriveSpeed);
					if(gyro.getAngle() < -88) {
						drive(0.0, 0.0);
						auton_atTarget = true;
					}
				}
			} else {
				// Spit out crate
				pickup(pickupSpeed * -1);
			}
		}
	}

	public void autonomousPeriodic()
	{
		switch(autonMode) {
			case 2: // Switch & Robot is on Left
				auton_sideLastSeen = "R";
			case 3: // Switch & Robot is on Right
				auton_sideLastSeen = "L";
				auton_placeInSwitch();
				break;
			case 4: // Scale & Robot is on Left
				auton_sideLastSeen = "R";
			case 5: // Scale & Robot is on Right
				auton_sideLastSeen = "L";
				auton_placeInScale();
				break;
			default:
				auton_driveStraightDistance(autonStraightOnlyDistance);
				break;
		}
	}

	public void teleopInit() {
		gyro.reset();
	}

	public void  teleopPeriodic() 
	{
		drive(left.getY(), right.getY());
		
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
		boolean rPLTLeft = right.getRawButton(11);
		boolean rPLTCenter = right.getRawButton(12);
		boolean rPLTRight = right.getRawButton(13);
		boolean rPLBLeft = right.getRawButton(16);
		boolean rPLBCenter = right.getRawButton(15);
		boolean rPLBRight = right.getRawButton(14);
		boolean rPRTLeft = right.getRawButton(7);
		boolean rPRTCenter = right.getRawButton(6);
		boolean rPRTRight = right.getRawButton(5);
		boolean rPRBLeft = right.getRawButton(8);
		boolean rPRBCenter = right.getRawButton(9);
		boolean rPRBRight = right.getRawButton(10);
		int rPOV = left.getPOV();  // Top Directional Thumb on joystick
		
		
		// Pickup Logic
		if(rTrigger) {
			pickup(pickupSpeed);
		} else {
			pickup(0.0);
		}

		// Stage 2 Logic
		if(lPOV == 0 || lPOV == 45 || lPOV == 315) {
			lift(stage2Speed);
		} else if(lPOV == 180 || rPOV == 285 || rPOV == 135) {
			lift(stage2Speed * -1);
		}

		// Climb logic
		if(lTrigger) {
			climbLock.set(.65);
		} else {
			climbLock.set(0);
		}

		// Carriage
		if(rPOV == 180 || rPOV == 285 || rPOV == 135) {
			carriage(carriageSpeed);
		}

		if(rPOV == 0 || lPOV == 45 || lPOV == 315) {
			carriage(carriageSpeed * -1);
		}
	}
}
