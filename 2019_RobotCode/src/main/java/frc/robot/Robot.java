
package frc.robot;

/** import packages and utilities */
import com.ctre.phoenix.motorcontrol.NeutralMode;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.DigitalInput;

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

import frc.robot.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Created by: Earl of March Lions Software Team
 * Created: January 2019
 * Last Edited: February 14 2019 <3 <3 <3
 * Desc: This code is the main code for our robot for the 2019 Deep Space game.
 * 
 * Other comments: Happy Valentines Day!!
 */

/**
 * Beginning of the Robot Class Extends the FRC TimedRobot robot class
 */
public class Robot extends TimedRobot {

	/** Hardware, either Talon could be a Victor */
	/** Initialize the hardware for the robot */
	WPI_VictorSPX _leftMasterFront = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftMasterBack = new WPI_VictorSPX(4);

	WPI_VictorSPX _rightMasterFront = new WPI_VictorSPX(1);
	WPI_VictorSPX _rightMasterBack = new WPI_VictorSPX(2);

	WPI_TalonSRX _elevator = new WPI_TalonSRX(5);
	WPI_TalonSRX _intake = new WPI_TalonSRX(7);

	/* limit switches */
	DigitalInput ballLimit = new DigitalInput(9);
	DigitalInput elevatorLimit = new DigitalInput(8);

	boolean moving = false;
	Joystick _gamepad = new Joystick(0);
	Encoder enc;
	int distance;
	edu.wpi.first.cameraserver.CameraServer server;
	boolean keepHeight = false;
	// Elevator elevator = new Elevator(_intake, 0.0);
	double speed = .2;
	ArrayList<Integer> prev = new ArrayList<>();

	@Override
	public void robotInit() {
		_elevator.getSensorCollection().setQuadraturePosition(0, 30);
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

		_elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 30);
		_elevator.getSensorCollection().setQuadraturePosition(0, 30);

		cameraInit();
	}

	@Override
	public void teleopPeriodic() {

		// /* Gamepad processing */
		// double forward = -1 * _gamepad.getY();
		// double turn = _gamepad.getX();
		// // System.out.println(forward);
		// int tick = _elevator.getSelectedSensorPosition();

		// System.out.println(tick);

		// if (_gamepad.getTrigger()) {
		// 	forward = Deadband(forward);
		// 	turn = Deadband(turn);
		// 	turn = Turn_Scale(forward, turn);
		// } else {
		// 	forward = Deadband_Scale(forward);
		// 	turn = Deadband_Scale(turn);
		// 	turn = Turn_Scale(forward, turn);
		// }

		// // if (_gamepad.getRawButton(5) && !ballLimit.get())
		// // _intake.set(ControlMode.PercentOutput, 1);

		// if (elevatorLimit.get())
		// 	_elevator.getSensorCollection().setQuadraturePosition(0, 30);

		// _intake.set(ControlMode.PercentOutput, turn);

		// if(forward == 0){
		// 	forward = 0.1;
		// }
		// else if (forward < 0.08 && tick < 1500) {
		// 	forward = .07;

		// } else if (forward < 0.08) {
		// 	if (prev.get(prev.size() - 1) == tick) {
		// 		forward = .0370;

		// 		System.out.println(prev.get(prev.size() - 1));
		// 	} else {
		// 		forward = .0378;
		// 	}

		// }
		// prev.add(tick);
		// System.out.println(forward);
		// _elevator.set(ControlMode.PercentOutput, -forward);

		// int position_wanted = 1000;
		// int position_reduce = 0;
		// position_wanted -= position_reduce;
		// if (moving) {
		// if (tick < (position_wanted -10) || tick > (position_wanted +10) ) {
		// System.out.println("Button 3 run Big if");
		// if (tick < position_wanted) {
		// System.out.println("Button 3 run Big minus speed");
		// _elevator.set(ControlMode.PercentOutput, -speed);
		// System.out.println("Tick is " + (tick));
		// }
		// else if (tick > position_wanted) {
		// System.out.println("Button 3 run Big Plus speed");
		// _elevator.set(ControlMode.PercentOutput, +speed);
		// System.out.println("Tick is " + (tick));
		// }
		// }
		// else {
		// moving = false;
		// }
		// }
		// else {

		// if (_gamepad.getRawButton(3) && tick != position_wanted) {
		// System.out.println("Button 3 run");

		// moving = true;
		// }
		// else if (_gamepad.getRawButton(4) && tick > -position_wanted) {
		// System.out.println("Button 4 run");
		// _elevator.set(ControlMode.PercentOutput, speed);
		// }
		// else {
		// System.out.println("Button 3 no run");
		// System.out.println("Button 4 no run");
		// _elevator.set(ControlMode.PercentOutput, 0);
		// }
		// _elevator.set(ControlMode.PercentOutput, forward);
		// }

		// if (_gamepad.getTrigger()) {
		// forward = Deadband(forward);
		// turn = Deadband(turn);
		// turn = Turn_Scale(forward, turn);
		// }
		// else {
		// forward = Deadband_Scale(forward);
		// turn = Deadband_Scale(turn);
		// turn = Turn_Scale(forward, turn);
		// }

		// /* Arcade Drive using PercentOutput along with Arbitrary Feed Forward
		// supplied by turn */
		// _leftMasterFront.set(ControlMode.PercentOutput, +turn,
		// DemandType.ArbitraryFeedForward, +forward);
		// _leftMasterBack.set(ControlMode.PercentOutput, +turn,
		// DemandType.ArbitraryFeedForward, +forward);

		// _rightMasterFront.set(ControlMode.PercentOutput, turn,
		// DemandType.ArbitraryFeedForward, -forward);
		// _rightMasterBack.set(ControlMode.PercentOutput, turn,
		// DemandType.ArbitraryFeedForward, -forward);



		
	}

	double Turn_Scale(double forwardValue, double turnValue) {
		if (forwardValue > 0.5)
			return turnValue *= (1 - forwardValue);
		else if (forwardValue < -0.5)
			return turnValue *= (1 + forwardValue);

		return turnValue;
	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband_Scale(double value) {
		/* Upper deadband */
		if (value >= +0.05)
			return value / 2;

		/* Lower deadband */
		if (value <= -0.05)
			return value / 2;

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

	public void cameraInit() {
		server = edu.wpi.first.cameraserver.CameraServer.getInstance();
		server.startAutomaticCapture(0);
		server.startAutomaticCapture(1);
	}

}
