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

public class Robot extends TimedRobot{
	
	
	WPI_TalonSRX m_frontLeft;
	WPI_TalonSRX m_rearLeft;
	WPI_TalonSRX m_frontRight;
	WPI_TalonSRX m_rearRight; 
		
	Joystick left, right;
	
	
	//drive
		//encoders
		Encoder drive_left = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		Encoder drive_right = new Encoder(2, 3, false, Encoder.EncodingType.k4X);	
	//Elevator
		//stage 2
			WPI_TalonSRX stage2Right = new WPI_TalonSRX(5);
			WPI_TalonSRX stage2Left= new WPI_TalonSRX(6);
			//switches
				DigitalInput stage2TopStop = new DigitalInput(4);
				DigitalInput stage2BottomStop = new DigitalInput(5);			
	//carriage
		//motor
			WPI_TalonSRX cariage = new WPI_TalonSRX(7);
				//encoder?
					Encoder drive_cariage = new Encoder(6,7, false, Encoder.EncodingType.k4X);
	//pickup
		//Motors must run together
		WPI_TalonSRX pickupRight = new WPI_TalonSRX(8);
		WPI_TalonSRX pickupLeft = new WPI_TalonSRX(9);
		//sensor
			DigitalInput cubePresent = new DigitalInput(8);
	//climb
		//lock
			Servo climbLock = new Servo(0);
			
	//speeds
			double pickupSpeed = 1.0;
			double stage2Speed = 1.0;
			double cariageSpeed = 1.0;
	
    public void robotInit() 
    {
    	m_frontLeft = new WPI_TalonSRX(1);
    	m_rearLeft = new WPI_TalonSRX(2);

    	m_frontRight = new WPI_TalonSRX(3);
    	m_rearRight = new WPI_TalonSRX(4);

    	left = new Joystick(0);
    	right = new Joystick(1);
    	CameraServer.getInstance().startAutomaticCapture(1);
    }

    public void autonomousPeriodic() {

    }

    public void  teleopPeriodic() 
    {
    	
    	
    	m_frontLeft.set(ControlMode.PercentOutput, left.getY());
    	m_rearLeft.set(ControlMode.PercentOutput, left.getRawAxis(1));
    	m_rearRight.set(ControlMode.PercentOutput, right.getRawAxis(1));
    	m_frontRight.set(ControlMode.PercentOutput, right.getRawAxis(1));
    	
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
		int lPOV = left.getPOV();
		
		
		//Right
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
		int rPOV = left.getPOV();
	
		
			//pickup logic
			if(rTrigger) 
			{
				if(cubePresent.get())
				{
					 pickupRight.set(pickupSpeed * -1);
					 pickupLeft.set(pickupSpeed * -1);
					 }
				else{
					pickupRight.set(pickupSpeed);
					pickupLeft.set(pickupSpeed);}}
			
	//stage2 logic
			if(lPOV == 0 || lPOV == 45 || lPOV == 315  && !stage2BottomStop.get())
			{
				stage2Right.set(stage2Speed);
				stage2Left.set(stage2Speed);
			}
			if(lPOV == 180 || rPOV == 285 || rPOV == 135 && !stage2TopStop.get())
			{
				stage2Right.set(stage2Speed * -1);
				stage2Left.set(stage2Speed * -1);
			}
			if (lPOV == -1)
			{
				stage2Right.set(0.0);
				stage2Left.set(0.0);
			}
		
	//climb logic
			if(lTrigger)
			{
				climbLock.set(.65);
			}
			else
			{
				climbLock.set(0);
			}
	//carriage
			if(rPOV == 180 || rPOV == 285 || rPOV == 135)
			{
				cariage.set(cariageSpeed * 1);
			}
			if(rPOV == 0 || lPOV == 45 || lPOV == 315)
			{
				cariage.set(cariageSpeed);
			}
			
		
		}
    }

