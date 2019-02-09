package frc.robot;

import edu.wpi.first.wpilibj.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator implements PIDOutput {
        /** The motor that will be set based on the {@link PIDController} results. */
        public WPI_TalonSRX motor;
        private double previousOutput = 0.0;
        private double rampBand;
        private double output;
        

        /**
         * Constructor for a PID controlled motor, with a controllable multiplier.
         *
         * @param motor The motor being set.
         * @param rampBand The acceptable range for a motor change in one loop
         */
        public Elevator(WPI_TalonSRX motor, double rampBand) {
                this.motor = motor;
                this.rampBand = rampBand;

        }

        public void pidWrite(double pidInput) {
                
                if (Math.abs(pidInput - previousOutput) > rampBand) { //If the change is greater that we want
                        output = pidInput - previousOutput > 0 ? previousOutput + rampBand : previousOutput - rampBand; //set output to be the previousOutput adjusted to the tolerable band, while being aware of positive/negative
                }
                else {
                        output = pidInput;
                }
                motor.set(output);
                previousOutput = output;
        }
}