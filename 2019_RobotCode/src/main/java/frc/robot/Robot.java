
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.DemandType;

public class Robot extends TimedRobot {
    /** Hardware */
	TalonSRX _elevator = new TalonSRX(5);
	TalonSRX _hatch = new TalonSRX(6);
	TalonSRX _intake = new TalonSRX(7);
	Joystick _joy = new Joystick(0);
	Joystick _gamepad = new Joystick(1);

	WPI_VictorSPX _leftMasterFront = new WPI_VictorSPX(3);
	WPI_VictorSPX _leftMasterBack = new WPI_VictorSPX(4);

	WPI_VictorSPX _rightMasterFront = new WPI_VictorSPX(1);
	WPI_VictorSPX _rightMasterBack = new WPI_VictorSPX(2);
	
    /** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	
    /** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
	double targetPositionRotations;

	edu.wpi.first.cameraserver.CameraServer server;

	public void robotInit() {
		/* Config the sensor used for Primary PID and sensor direction */
        _elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
                                            Constants.kPIDLoopIdx,
				                            Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		_elevator.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		_elevator.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		_elevator.configNominalOutputForward(0, Constants.kTimeoutMs);
		_elevator.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_elevator.configPeakOutputForward(1, Constants.kTimeoutMs);
		_elevator.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_elevator.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		_elevator.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_elevator.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_elevator.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_elevator.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = _elevator.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase) { absolutePosition *= -1; }
		if (Constants.kMotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		_elevator.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_elevator.getSensorCollection().setQuadraturePosition(0, 30);

		
    }
    
	void commonLoop() {
		/* Gamepad processing */
		double turn = _joy.getX()/2;
		double forward = -1 * _joy.getY()/2;
		
		boolean button2 = _joy.getRawButton(2);
		boolean button3 = _joy.getRawButton(3);
		boolean button4 = _joy.getRawButton(4);
		boolean button5 = _joy.getRawButton(5);	
		boolean button6 = _joy.getRawButton(6);
		boolean button7 = _joy.getRawButton(7);	
		boolean button8 = _joy.getRawButton(8);
		boolean button9 = _joy.getRawButton(9);	
		boolean button10 = _joy.getRawButton(10);
		boolean button11 = _joy.getRawButton(11);
		boolean button12 = _joy.getRawButton(12);
		
		boolean game_button1 = _gamepad.getRawButton(1);
		boolean game_button2 = _gamepad.getRawButton(2);
		boolean game_button3 = _gamepad.getRawButton(3);
		boolean game_button4 = _gamepad.getRawButton(4);
		boolean game_button5 = _gamepad.getRawButton(5);
		boolean game_button6 = _gamepad.getRawButton(6);
		boolean game_button7 = _gamepad.getRawButton(7);
		boolean game_button8 = _gamepad.getRawButton(8);


		/* Get Talon/Victor's current output percentage */
		double motorOutput = _elevator.getMotorOutputPercent();

		int tick = -1 *_elevator.getSelectedSensorPosition();

		System.out.println(tick);


		/* Deadband gamepad */
		if (Math.abs(forward) < 0.10) {
			/* Within 10% of zero */
			forward = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(_elevator.getSelectedSensorPosition(0));
		_sb.append("u"); 	// Native units

	
		
		if (game_button8) {
			//Second Cargo ball
			targetPositionRotations = -4200;
			_elevator.set(ControlMode.Position, targetPositionRotations);
		}
		if (game_button6) {
			// First Cargo ball
			targetPositionRotations = -1650;
			_elevator.set(ControlMode.Position, targetPositionRotations);
		}

		if (game_button5) {
			//Second Cargo Hatch
			targetPositionRotations = -540;
			_elevator.set(ControlMode.Position, targetPositionRotations);
		}
		if (game_button7) {
			// First Cargo hatch
			targetPositionRotations = -2920;
			_elevator.set(ControlMode.Position, targetPositionRotations);
		}

		if (game_button4) {
			//Sets location to 0.
			_elevator.set(ControlMode.Current, 0);
		}

		if (game_button3){
			//target hatch level one
			targetPositionRotations = _elevator.getSelectedSensorPosition(0);
			_elevator.set(ControlMode.Position, targetPositionRotations);
		}

		if (game_button1)
		{
			//target hatch level two
			targetPositionRotations = 0;
			_elevator.set(ControlMode.PercentOutput, -0.55);
		}

		if (game_button2)
		{
			//target hatch level three
			targetPositionRotations = 0;
			_elevator.set(ControlMode.PercentOutput, 0.05);
		}


		/* If Talon is in position closed-loop, print some more info */
		if (_elevator.getControlMode() == ControlMode.Position) {
			/* ppend more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_elevator.getClosedLoopError(0));
			_sb.append("u");	// Native Units

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u");	// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
		
		_leftMasterFront.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, +forward);
		_leftMasterBack.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, +forward);

		_rightMasterFront.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, -forward);
		_rightMasterBack.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, -forward);

		if(button4){
			_intake.set(ControlMode.PercentOutput, 0.5);
		}		//Spins intake mechinism outwards
		else if(button3) {
			_intake.set(ControlMode.PercentOutput, -0.5);
		}		//Spins intake mechinism inwards
		else {
			_intake.set(ControlMode.PercentOutput, 0);
		}		//Stops intake mechinism motor

		if(button6) {
			_hatch.set(ControlMode.PercentOutput, 0.3);
		}		//Puts harch arm up
		
		else if(button5) {
			_hatch.set(ControlMode.PercentOutput, -0.3);
		}		//Puts hatch arm down
		else {
			_hatch.set(ControlMode.PercentOutput, 0);
		}		//Stops hatch arm

		
		cameraInit();
    }
    
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		commonLoop();
	}

	public void cameraInit() {
		server = edu.wpi.first.cameraserver.CameraServer.getInstance();
		server.startAutomaticCapture(0);
		// server.startAutomaticCapture(1);
	}

}