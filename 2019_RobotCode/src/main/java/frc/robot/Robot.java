
package frc.robot;

/** import packages and utilities */
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;

import frc.robot.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/**
 * Created by: Earl of March Lions Software Team
 * Created: January 2019
 * Last Edited: February 14 2019 <3 <3 <3
 * Desc: This code is the main code for our robot for the 2019 Deep Space game.
 * 
 * February 14th Comment from A-Aron: Happy Valentine's Day Everyone!! <3 <3 <3
 * Feb 14 Comment from Laura: I want a boyfriend. Happy Singles Day. 
 */

/** 
 * Beginning of the Robot Class
 * Extends the FRC TimedRobot robot class 
 */


public class Robot extends TimedRobot {

	/** Hardware, either Talon could be a Victor */
	/** Initialize the victors for the robot's motors */
	WPI_VictorSPX _leftMasterFront = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftMasterBack = new WPI_VictorSPX(4);
	WPI_VictorSPX _rightMasterFront = new WPI_VictorSPX(1);
	WPI_VictorSPX _rightMasterBack = new WPI_VictorSPX(2);

	/** initialize the intake for the quadencoder */
	WPI_TalonSRX _intake = new WPI_TalonSRX(8);

	/** initialize a boolean that records if the previous instance of the robot is moving */
	boolean isMoving = false;

	/** initialize the gamepad for the Logitech EXTREME 3D PRO*/
	Joystick _gamepad = new Joystick(0);

	edu.wpi.first.cameraserver.CameraServer server;

	/** initialize an elevator for the robot */
	Elevator elevator = new Elevator(_intake, 0.0);


	/** Beginning of the robotInit class to initialize robot class */
	@Override
	public void robotInit() {
		_intake.getSensorCollection().setQuadraturePosition(0, 30);
	} // end of robotInit method


	/** Beginning of the telopInit class */
	@Override
	public void teleopInit() {
		/** Ensure motor output is neutral during init */
		_leftMasterFront.set(ControlMode.PercentOutput, 0);
		_leftMasterBack.set(ControlMode.PercentOutput, 0);
		_rightMasterFront.set(ControlMode.PercentOutput, 0);
		_rightMasterBack.set(ControlMode.PercentOutput, 0);
		
		/** set all robots configurations to factory default to prevent unexpected behaviour */
		_leftMasterFront.configFactoryDefault();
		_leftMasterBack.configFactoryDefault();
		_rightMasterFront.configFactoryDefault();
		_rightMasterBack.configFactoryDefault();
		
		/** Set the motors to neutral mode */
		_leftMasterFront.setNeutralMode(NeutralMode.Brake);
		_leftMasterBack.setNeutralMode(NeutralMode.Brake);
		_rightMasterFront.setNeutralMode(NeutralMode.Brake);
		_rightMasterBack.setNeutralMode(NeutralMode.Brake);
			
		/** Configure output direction */
		_leftMasterFront.setInverted(false);
		_leftMasterBack.setInverted(false);
		_rightMasterFront.setInverted(true);
		_rightMasterBack.setInverted(true);

		/** Configure the intake to intake the QuadEncoder Data */
		_intake.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 30);
		_intake.getSensorCollection().setQuadraturePosition(0, 30);
		
	} // end of the teleopInit method
	

	/** Beginning of the teleopPeriodic method */
	@Override
	public void teleopPeriodic() {
		
		/* Receive the current position of the remote and record them */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getX();		
		
		/**
		 *
		 *  >> IMPORTANT <<
		 * the tick does not work properly, at least not yet
		 * 
		 */
		int tick = _intake.getSelectedSensorPosition();
		
		System.out.println(tick);


		/**
		 * >> IMPORTANT QUERY << 
		 * Why is this in an if statement if the code is going to be run either way
		 */
		/** get the current value of the gamepad */
		if (_gamepad.getTrigger()) {
			/** 
			 * run the variables through the deadband method to ensure
			 * that the motors don't run even when the gamepad is at rest 
			 */
			forward = Deadband(forward);
			turn = Deadband(turn);
			turn = Turn_Scale(forward, turn);
		} else {
			forward = Deadband_Scale(forward);
			turn = Deadband_Scale(turn);
			turn = Turn_Scale(forward, turn);
		}
		
		/**
		 * >> IMPORTANT <<
		 * As this is a large section, it may be best to move it off to another method alltogether
		 * This will make the code much more readable
		 */

		/** Beginning of the movement of the intake quadcoder section */
		int	position_wanted = 80;
		position_wanted = 80;
		if(isMoving) {

			if (tick < position_wanted){
				System.out.println("Button 3 run Big if");
				_intake.set(ControlMode.PercentOutput, -.05);
				System.out.println("Tick is " +(tick));
			}
			else{
				isMoving = false;
			}

		} else {
			//PIDController pidpod = new PIDController(.1,.001,0.0,tick,_intake);
			if (_gamepad.getRawButton(3) && tick < position_wanted) {
				System.out.println("Button 3 run");
				
				isMoving=true;
				
			} else if (_gamepad.getRawButton(4) && tick > -position_wanted) {
				System.out.println("Button 4 run");
				_intake.set(ControlMode.PercentOutput, .05);
				
			} else {
				System.out.println("Button 3 no run");
				System.out.println("Button 4 no run");
				_intake.set(ControlMode.PercentOutput, 0);
			}
			_intake.set(ControlMode.PercentOutput, forward);
		}


		if (_gamepad.getRawButton(12)) {
			System.out.println("Call Button Stuff");
			elevator.pidWrite(3);
		}

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		// _leftMasterFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		// _leftMasterBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		
		// _rightMasterFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		// _rightMasterBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
	} // end of the teleopPeriodic method


	/** 
	 * Please delete if you are not going to use this
	 */
	void move(double position) {
		
	}


	/**
	 * Beginning of the Turn_Scale method
	 * This method controls the turning value to ensure smooth turning and driving
	 */
	double Turn_Scale(double forwardValue, double turnValue) {
		if (forwardValue > 0.5)
			return turnValue *= (1 - forwardValue);
		else if (forwardValue < -0.5)
			return turnValue *= (1 + forwardValue);
		
		return turnValue;
	} // end of the deadband method


	/** Beginning of the deadband scale method. This method ensures that the resting controller returns coordinates (0,0) while halving the output */
	double Deadband_Scale(double value) {
		/** if the value received is less than 5% in either direction, return half of the value */
		if (value >= +0.05 || value <= -0.05)
			return value/2;
		
		/** if the value is in between -0.05 and 0.05, set the value to 0 */
		return 0;
	}


	/** Beginning of the deadband. This is the same as the deadband scale except without a scale */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	} // end of the Deadband


	/** Beginning of the cameraInit method that initializes the camera */
	public void cameraInit () {
		// server = edu.wpi.first.cameraserver.CameraServer.getInstance();
		// server.startAutomaticCapture(0);
		// server.startAutomaticCapture(1);	
	} // end of the cameraInit method

	

}

