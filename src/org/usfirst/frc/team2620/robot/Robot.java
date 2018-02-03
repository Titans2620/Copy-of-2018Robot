package org.usfirst.frc.team2620.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.CameraServer;

public class Robot extends TimedRobot {
	
	WPI_TalonSRX driveRight = new WPI_TalonSRX(1);
	WPI_TalonSRX driveLeft = new WPI_TalonSRX(5);

	WPI_TalonSRX stage2Right = new WPI_TalonSRX(5);
	WPI_TalonSRX stage2Left= new WPI_TalonSRX(6);
	WPI_TalonSRX cariage = new WPI_TalonSRX(7);
	WPI_TalonSRX pickupRight = new WPI_TalonSRX(8);
	WPI_TalonSRX pickupLeft = new WPI_TalonSRX(9);
	
	Servo climbLock = new Servo(0);
	
	Encoder drive_left = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder drive_right = new Encoder(2, 3, false, Encoder.EncodingType.k4X);	
	Encoder drive_cariage = new Encoder(4,5, false, Encoder.EncodingType.k4X);
	
	DigitalInput stage2TopStop = new DigitalInput(6);
	DigitalInput stage2BottomStop = new DigitalInput(7);
	DigitalInput cubePresent = new DigitalInput(8);			
	
	Joystick left = new Joystick(0);
	Joystick right = new Joystick(1);
	
	double pickupSpeed = 1.0;
	double stage2Speed = 1.0;
	double cariageSpeed = 1.0;

	// Auton Vars
	Timer autonTimer;
	SendableChooser autonChooser;
	int autonMode;
	String autonGameData;
	double autonDriveSpeed = 0.4; // In % Power
	double autonDriveTime = 10; // Game Time In Seconds (NOT RUNNING TIME)
	
	public void robotInit() {
		autonChooser = new SendableChooser();
		autonChooser.addDefault("Drive Forward ONLY", 1);
		autonChooser.addObject("Left & Target Possible", 2);
		autonChooser.addObject("Right & Target Possible", 3);
	}

	public void autonomousInit() {
		autonGameData = DriverStation.getInstance().getGameSpecificMessage();
		autonMode = (int) autonChooser.getSelected();
		autonTimer = new Timer();
	}

	public void autonomousPeriodic()
	{
		// void driveForwardOnly()
		// {
		// 	double speed = autonDriveSpeed;
		// 	if(autonTimer.getMatchTime() >= autonDriveTime) {
		// 		speed = 0.0;
		// 	}

		// 	driveRight.set(ControlMode.PercentOutput, speed);
		// 	driveLeft.set(ControlMode.PercentOutput, speed);
		// }

		// void placeInTarget() 
		// {
		// 	boolean shouldPlaceLeft = (autonGameData.charAt(0) == 'L' && autonMode == 2);
		// 	boolean shouldPlaceRight = (autonGameData.charAt(0) == 'R' && autonMode == 3);

		// 	if(shouldPlaceLeft) {
		// 		// Put left auto code here
		// 	} else if (shouldPlaceRight) {
		// 		// Put right auto code here
		// 	} else {
		// 		driveForwardOnly();
		// 	}
		// }

		// switch(autonMode) {
		// 	case 2:
		// 	case 3:
		// 		placeInTarget();
		// 		break;
		// 	default:
		// 		driveForwardOnly();
		// 		break;
		// }
	}

	public void  teleopPeriodic() 
	{
		driveRight.set(ControlMode.PercentOutput, left.getY());
		driveLeft.set(ControlMode.PercentOutput, right.getRawAxis(1));
		
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
			if(cubePresent.get()) {
				pickupRight.set(pickupSpeed * -1);
				pickupLeft.set(pickupSpeed * -1);
			} else {
				pickupRight.set(pickupSpeed);
				pickupLeft.set(pickupSpeed);
			}
		}

		// Stage 2 Logic
		if(lPOV == 0 || lPOV == 45 || lPOV == 315  && !stage2BottomStop.get()) {
			stage2Right.set(stage2Speed);
			stage2Left.set(stage2Speed);
		}

		if(lPOV == 180 || rPOV == 285 || rPOV == 135 && !stage2TopStop.get()) {
			stage2Right.set(stage2Speed * -1);
			stage2Left.set(stage2Speed * -1);
		}

		if (lPOV == -1) {
			stage2Right.set(0.0);
			stage2Left.set(0.0);
		}

		// Climb logic
		if(lTrigger) {
			climbLock.set(.65);
		} else {
			climbLock.set(0);
		}

		// Carriage
		if(rPOV == 180 || rPOV == 285 || rPOV == 135) {
			cariage.set(cariageSpeed * 1);
		}

		if(rPOV == 0 || lPOV == 45 || lPOV == 315) {
			cariage.set(cariageSpeed);
		}
	}
}
