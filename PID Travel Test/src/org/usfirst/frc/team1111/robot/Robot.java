/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1111.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	Encoder encoderFrontLeft = new Encoder(0, 1, false, EncodingType.k4X);
	int encoderPPR = 1; //TODO: Configure
	double encoderDPP = calculateEncoderDPP();
	
	double p = 0, i = 0, d = 0;
//	PIDController pid = new PIDController(p, i, d, encoderFrontLeft, this);
	MiniPID pid = new MiniPID(p, i, d);
	
	double speed;
	WPI_TalonSRX motorFrontLeft = new WPI_TalonSRX(11); //TODO: Configure
	WPI_TalonSRX motorFrontRight = new WPI_TalonSRX(11); //TODO: Configure
	WPI_TalonSRX motorBackLeft = new WPI_TalonSRX(11); //TODO: Configure
	WPI_TalonSRX motorBackRight = new WPI_TalonSRX(11); //TODO: Configure
	
	double testDist = 0;
	
	@Override
	public void robotInit() {
//		//Sets the feedback device of the motor controller to a magnetic quadrature encoder with absolute positioning. 
//		//0 specifies Primary control loop and 200 is the timeout delay
//		motorFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
		SmartDashboard.putNumber("Proportion:", p);
		SmartDashboard.putNumber("Integral:", i);
		SmartDashboard.putNumber("Derivative:", d);
		SmartDashboard.putNumber("Speed:", speed);
		SmartDashboard.putNumber("Test Distance:", testDist);
		
		pid.setOutputLimits(-1, 1);
		
		encoderFrontLeft.setDistancePerPulse(encoderDPP);
	}
	
	@Override
	public void teleopInit() {
		encoderFrontLeft.reset();
		p = SmartDashboard.getNumber("Proportion:", p);
		i = SmartDashboard.getNumber("Integral:", i);
		d = SmartDashboard.getNumber("Derivative:", d);
		testDist = SmartDashboard.getNumber("Test Distance:", testDist);
		pid.setPID(p, i, d);
	}

	@Override
	public void teleopPeriodic() {
		driveDistance(testDist);
	}

	public void driveDistance(double dist) {
		double dz = .5; //Configure to create a deadzone +/- the set value. Helps to prevent oscillation and accounts for drift
		
		pid.setSetpoint(dist);
		double distTraveled = encoderFrontLeft.getDistance();
		speed = pid.getOutput(distTraveled);
		SmartDashboard.putNumber("Speed:", speed);
		
		if (dist-dz < distTraveled && distTraveled < dist+dz) { //Checks if distance traveled is in the deadzone of desired distance
			drive(0);
		}
		else {
			drive(speed);
		}
	}
	
	public double calculateEncoderDPP() {
		double diameter = 1; //TODO: Configure with actual diameter of robot wheels in inches
		return Math.PI*diameter / encoderPPR; //NOTE: This does NOT factor in a gearbox!
	}
	
	public void drive(double speed) {
		double limiter = 1; //Configure to set hard limits on motor speed
		
		motorFrontLeft.set(speed*limiter);
		motorFrontRight.set(speed*limiter);
		motorBackLeft.set(speed*limiter);
		motorBackRight.set(speed*limiter);
	}
}
