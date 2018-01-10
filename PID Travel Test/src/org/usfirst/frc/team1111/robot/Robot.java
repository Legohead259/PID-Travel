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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput {
	Encoder encoderFrontLeft = new Encoder(0, 1, false, EncodingType.k4X);
	int encoderPPR = 1; //TODO: Configure
	double encoderDPP = calculateEncoderDPP();	
	double pEnc = 0, iEnc = 0, dEnc = 0;
	MiniPID encPID = new MiniPID(pEnc, iEnc, dEnc);
	
//	double pNav = 0, iNav = 0, dNav = 0;
//	PIDController navPID = new PIDController(pEnc, iEnc, dEnc, navx, this);
//	AHRS navx = new AHRS(...); //TODO: Configure with proper navX device
	
	double speed;
	WPI_TalonSRX motorFrontLeft = new WPI_TalonSRX(51); //TODO: Configure
	WPI_TalonSRX motorFrontRight = new WPI_TalonSRX(62); //TODO: Configure
	WPI_TalonSRX motorBackLeft = new WPI_TalonSRX(58); //TODO: Configure
	WPI_TalonSRX motorBackRight = new WPI_TalonSRX(45); //TODO: Configure
	
	double testDist = 0;
	
	@Override
	public void robotInit() {
//		//Sets the feedback device of the motor controller to a magnetic quadrature encoder with absolute positioning. 
//		//0 specifies Primary control loop and 200 is the timeout delay
//		motorFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
		SmartDashboard.putNumber("Proportion:", pEnc);
		SmartDashboard.putNumber("Integral:", iEnc);
		SmartDashboard.putNumber("Derivative:", dEnc);
		SmartDashboard.putNumber("Speed:", speed);
		SmartDashboard.putNumber("Test Distance:", testDist);
		
		encPID.setOutputLimits(-1, 1);
		
		encoderFrontLeft.setDistancePerPulse(encoderDPP);
	}
	
	@Override
	public void teleopInit() {
		encoderFrontLeft.reset();
		pEnc = SmartDashboard.getNumber("Proportion:", pEnc);
		iEnc = SmartDashboard.getNumber("Integral:", iEnc);
		dEnc = SmartDashboard.getNumber("Derivative:", dEnc);
		testDist = SmartDashboard.getNumber("Test Distance:", testDist);
		encPID.setPID(pEnc, iEnc, dEnc);
	}

	@Override
	public void teleopPeriodic() {
		driveDistance(testDist);
	}

	public void driveDistance(double dist) {
		double dz = .5; //Configure to create a deadzone +/- the set value. Helps to prevent oscillation and accounts for drift
		
		encPID.setSetpoint(dist);
		double distTraveled = encoderFrontLeft.getDistance();
		speed = encPID.getOutput(distTraveled);
		SmartDashboard.putNumber("Speed:", speed);
		
		if (dist-dz < distTraveled && distTraveled < dist+dz) { //Checks if distance traveled is in the deadzone of desired distance
			drive(0);
		}
		else {
			drive(speed);
		}
	}
	
//	public void turnTo(double angle) {
//		double curAngle = navx.getAngle();
//		double dz = 1; //Configure to create a deadzone +/- the set value. Helps prevent oscillation and accounts for drift
//		
//		if (angle-dz < curAngle && curAngle < angle+dz) { //Checks if distance traveled is in the deadzone of desired distance
//			drive(0);
//		}
//		else {
//			//Checks if the robot needs to turn left or right
//			if (curAngle-angle > 0) { //Checks if needs to turn left 
//				drive(-speed, speed);
//			}
//			else {
//				drive(speed, -speed);
//			}
//		}
//	}
	
	public double calculateEncoderDPP() {
		double diameter = 4; //TODO: Configure with actual diameter of robot wheels in inches
		return Math.PI*diameter / encoderPPR; //NOTE: This does NOT factor in a gearbox!
	}
	
	public void drive(double speed) {
		double limiter = 1; //Configure to set hard limits on motor speed
		
		motorFrontLeft.set(speed*limiter);
		motorFrontRight.set(speed*limiter);
		motorBackLeft.set(speed*limiter);
		motorBackRight.set(speed*limiter);
	}
	
	public void drive(double speedL, double speedR) {
		double limiter = 1; //Configure to set hard limits on motor speed
		
		motorFrontLeft.set(speedL*limiter);
		motorFrontRight.set(speedR*limiter);
		motorBackLeft.set(speedL*limiter);
		motorBackRight.set(speedR*limiter);
	}
	
	public void pidWrite(double output) {
		speed = output;
		SmartDashboard.putNumber("Speed:", speed);
	}
}
