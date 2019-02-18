package frc.robot;

import edu.wpi.first.wpilibj.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class elevator extends PIDSubsystem {
        /** The motor that will be set based on the {@link PIDController} results. */
        public WPI_TalonSRX motor;
        private double previousOutput = 0.0;
        private double rampBand;
        private double output;
        public Encoder enc;

        /**
         * Constructor for a PID controlled motor, with a controllable multiplier.
         *
         * @param motor The motor being set.
         * @param rampBand The acceptable range for a motor change in one loop
         */

     
        public elevator() {
                super("Elevator", 1.0, 0.0, 0.0);
                setAbsoluteTolerance(0.2);
                this.motor = motor;

        }
        
        public void initDefaultCommand() {

        }

        protected double returnPIDInput() {
                return motor.getSelectedSensorPosition();
        }

        protected void usePIDOutput(double output) {
                motor.pidWrite(output);
        }
}