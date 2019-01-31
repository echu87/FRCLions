package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.CameraServer;
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
//import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;



public class Robot extends TimedRobot {

	/** Hardware, either Talon could be a Victor */
	WPI_VictorSPX _leftMasterFront = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftMasterBack = new WPI_VictorSPX(4);

	WPI_VictorSPX _rightMasterFront = new WPI_VictorSPX(1);
	WPI_VictorSPX _rightMasterBack = new WPI_VictorSPX(2);
  
	Joystick _gamepad = new Joystick(0);

	edu.wpi.first.cameraserver.CameraServer server;

	@Override
	public void robotInit() {
		
	}

	@Override
	public void teleopInit() {
		/* Ensure motor output is neutral during init */
		_leftMasterFront.set(ControlMode.PercentOutput, 0);
		_leftMasterBack.set(ControlMode.PercentOutput, 0);
		_rightMasterFront.set(ControlMode.PercentOutput, 0);
		_rightMasterBack.set(ControlMode.PercentOutput, 0);
		
		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMasterFront.configFactoryDefault();
		_leftMasterBack.configFactoryDefault();
		_rightMasterFront.configFactoryDefault();
		_rightMasterBack.configFactoryDefault();
		
		/* Set Neutral mode */
		_leftMasterFront.setNeutralMode(NeutralMode.Brake);
		_leftMasterBack.setNeutralMode(NeutralMode.Brake);
		_rightMasterFront.setNeutralMode(NeutralMode.Brake);
		_rightMasterBack.setNeutralMode(NeutralMode.Brake);
			
		/* Configure output direction */
		_leftMasterFront.setInverted(false);
		_leftMasterBack.setInverted(false);
		_rightMasterFront.setInverted(true);
		_rightMasterBack.setInverted(true);
		
		cameraInit();

		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
	}
	
	@Override
	public void teleopPeriodic() {
		
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		
		double turn = _gamepad.getX();		
			
		forward = Deadband(forward);
		turn = Deadband(turn);
		System.out.println(turn);
		// System.out.println(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		_leftMasterFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		_leftMasterBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		
		_rightMasterFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		_rightMasterBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband_Scale(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value/2;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value/2;
		
		/* Outside deadband */
		return 0;
	}

	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;

	}

	public void cameraInit () {
		server = edu.wpi.first.cameraserver.CameraServer.getInstance();
		server.startAutomaticCapture(0);
		server.startAutomaticCapture(1);		
	}

}

