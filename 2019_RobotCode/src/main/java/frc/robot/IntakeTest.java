
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
//import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;

import frc.robot.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
/**
 * Created by: Earl of March Lions Software Team
 * Created: January 2019
 * Last Edited: February 14 2019 <3 <3 <3
 * Desc: This code is the main code for our robot for the 2019 Deep Space game.
 * 
 * Other comments: Happy Valentines Day!!
 */

/** Beginning of the Robot Class
 *  Extends the FRC TimedRobot robot class 
 */
public class IntakeTest extends TimedRobot {

	/** Hardware, either Talon could be a Victor */
	/** Initialize the hardware for the robot */

    WPI_TalonSRX _ball = new WPI_TalonSRX(9);
    DigitalInput limitSwitch = new DigitalInput(9);

	boolean moveing = false;
	Joystick _gamepad = new Joystick(0);
	Encoder enc;
	int distance;
	edu.wpi.first.cameraserver.CameraServer server;
	boolean keepHeight = false;
	//Elevator elevator = new Elevator(_intake, 0.0);
    double speed = .2;
    double ballSpeed = .2;

	@Override
	public void robotInit() {
		//_intake.getSensorCollection().setQuadraturePosition(0, 30);
	}

	@Override
	public void teleopInit() {
		/* Ensure motor output is neutral during init */
		// _leftMasterFront.set(ControlMode.PercentOutput, 0);
		// _leftMasterBack.set(ControlMode.PercentOutput, 0);
		// _rightMasterFront.set(ControlMode.PercentOutput, 0);
		// _rightMasterBack.set(ControlMode.PercentOutput, 0);
		
		// /* Factory Default all hardware to prevent unexpected behaviour */
		// _leftMasterFront.configFactoryDefault();
		// _leftMasterBack.configFactoryDefault();
		// _rightMasterFront.configFactoryDefault();
		// _rightMasterBack.configFactoryDefault();
		
		// /* Set Neutral mode */
		// _leftMasterFront.setNeutralMode(NeutralMode.Brake);
		// _leftMasterBack.setNeutralMode(NeutralMode.Brake);
		// _rightMasterFront.setNeutralMode(NeutralMode.Brake);
		// _rightMasterBack.setNeutralMode(NeutralMode.Brake);
			
		// /* Configure output direction */
		// _leftMasterFront.setInverted(false);
		// _leftMasterBack.setInverted(false);
		// _rightMasterFront.setInverted(true);
		// _rightMasterBack.setInverted(true);

		// _intake.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 30);
		// _intake.getSensorCollection().setQuadraturePosition(0, 30);
		
		//cameraInit();
		
	}
	
	@Override
	public void teleopPeriodic() {
		
        if (_gamepad.getRawButton(5) && !limitSwitch.get())
        {
            System.out.println(limitSwitch.get());
            _ball.set(ControlMode.PercentOutput, ballSpeed);
        }
        if (_gamepad.getRawButton(6))
        {
            _ball.set(ControlMode.PercentOutput, -ballSpeed);
        }

        if (_gamepad.getRawButton(12)) 
        {
			System.out.println("Call Button Stuff");
			//elevator.pidWrite(3);
		}
		


		

	}


	void move(double position) {
		
	
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
		// server = edu.wpi.first.cameraserver.CameraServer.getInstance();
		// server.startAutomaticCapture(0);
		// server.startAutomaticCapture(1);	
	}

	

}