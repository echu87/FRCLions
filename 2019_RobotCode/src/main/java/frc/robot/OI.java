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
import edu.wpi.first.wpilibj.buttons.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.JoystickBase;

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

public class OI {

Joystick joy = new Joystick(0);

JoystickButton button5 = new JoystickButton(joy, 5),
			button6 = new JoystickButton(joy, 6),
			button7 = new JoystickButton(joy, 7),
			button8 = new JoystickButton(joy, 8),
			button9 = new JoystickButton(joy, 9),
			button10 = new JoystickButton(joy, 10),
			button11 = new JoystickButton(joy, 11),
            button12 = new JoystickButton(joy, 12);
            
    public OI() {
        button5.whenPressed()



    }
}