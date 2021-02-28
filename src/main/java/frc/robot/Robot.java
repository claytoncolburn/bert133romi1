// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class Robot extends TimedRobot {
	static final String kDefaultAuto = "Default";
	static final String kCustomAuto = "My Auto";
	String m_autoSelected;
	final SendableChooser<String> m_chooser = new SendableChooser<>();

	final RomiDrivetrain m_drivetrain = new RomiDrivetrain();

	XboxController xboxController = new XboxController(0);

	double driveY = 0.0;
	double driveX = 0.0;

	double leftEncoder = 0.0;
	double rightEncoder = 0.0;

	final double DEAD_ZONE = 0.2;
	final double SPEED_SCALE = 0.9;
	final double TURN_SCALE = 0.8;
	final double TURN_SPEED_MIN = 0.3;
	final double GRYO_BOUNDS = 2.0;
	final double TURNING_FACTOR = 0.04844;

	SimDouble gyro;
	double gyroOffset = 0.0;
	double gyroTurningScale = 90.0;
	double targetEncoder = 0.0;

	int autoStage = 1;

	boolean turningPressed = false;
	boolean driveStraightPressed = false;

	Timer timer = new Timer();

	@Override
	public void robotInit() {
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		m_chooser.addOption("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		SimDevice gyroSimDevice = SimDevice.create("Gyro:RomiGyro");

		if (gyroSimDevice != null) {
			gyroSimDevice.createBoolean("init", Direction.kOutput, true);
			gyro = gyroSimDevice.createDouble("angle_z", Direction.kInput, 0.0);
		}

		timer.start();
	}

	@Override
	public void robotPeriodic() {
		leftEncoder = m_drivetrain.getLeftDistanceInch();
		rightEncoder = m_drivetrain.getRightDistanceInch();

		SmartDashboard.putNumber("Drive Y", driveY);
		SmartDashboard.putNumber("Drive X", driveX);
		SmartDashboard.putNumber("Left Encoder", leftEncoder);
		SmartDashboard.putNumber("Right Encoder", rightEncoder);
		SmartDashboard.putNumber("Angle", getGyroAngle());
		SmartDashboard.putNumber("Raw Angle", gyro.get());
		SmartDashboard.putNumber("Gryo Offset", gyroOffset);
		SmartDashboard.putNumber("Gryo Turning Scale", gyroTurningScale);
		SmartDashboard.putNumber("Auto Stage", autoStage);
	}

	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);

		m_drivetrain.resetEncoders();
		gyroOffset = gyro.get();

		autoStage = 1;

		driveY = 0;
		driveX = 0;

		timer.reset();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		if (timer.get() < 1.0) {
			return;
		}

		switch(autoStage) {
			case 1:
			drive(13);
			break;
			case 2:
			turn(-70);
			break;
			case 3:
			drive(13.5);
			break;
			case 4:
			turn(-88);
			break;
			case 5:
			drive(6);
			break;
			case 6:
			turn(-40);
			break;
			case 7:
			drive(7);
			break;
			case 8:
			turn(45);
			break;
			case 9:
			drive(7);
			break;
			case 10:
			turn(100);
			break;
			case 11:
			drive(13.5);
			break;
			case 12:
			turn(95);
			break;
			case 13:
			drive(14);
			break;
			default:
			driveY = 0.0;
			driveX = 0.0;
			break;
		}

		m_drivetrain.arcadeDrive(driveY, driveX);
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		if (xboxController.getYButton()) {
			if (driveStraightPressed == false) {
				gyroOffset = gyro.get();
				driveStraightPressed = true;
			}

			driveY = 0.8;
		} else if ((Math.abs(xboxController.getY(Hand.kLeft)) > DEAD_ZONE) &&
			!xboxController.getBButton() &&
			!xboxController.getXButton()){

			driveY = -xboxController.getY(Hand.kLeft) * SPEED_SCALE;
			driveStraightPressed = false;
		} else {
			driveY = 0.0;
			driveStraightPressed = false;
		}

		if (xboxController.getYButton()) {
			driveX = -(getGyroAngle() / 20);
		} else if (xboxController.getBButton()) {
			if (turningPressed == false) {
				gyroOffset = gyro.get();
				m_drivetrain.resetEncoders();
				targetEncoder = TURNING_FACTOR * 90;
				turningPressed = true;
			}

			if ((getGyroAngle() - 90.0) > -10.0) {
				driveX = 0;
			} else {
				driveX = -(((getGyroAngle() - 90.0) / 90.0) * 0.25) + 0.25;
			}

		} else if (xboxController.getXButton()) {
			if (turningPressed == false) {
				gyroOffset = gyro.get();
				m_drivetrain.resetEncoders();
				targetEncoder = TURNING_FACTOR * -90;
				turningPressed = true;
			}

			if ((getGyroAngle() + 90.0) < 10.0) {
				driveX = 0;
			} else {
				driveX = -(((getGyroAngle() + 90.0) / 90.0) * 0.25) - 0.25;
			}
		} else if (Math.abs(xboxController.getX(Hand.kRight)) > DEAD_ZONE) {
			driveX = xboxController.getX(Hand.kRight) * TURN_SCALE;
			turningPressed = false;
		} else {
			driveX = 0.0;
			turningPressed = false;
		}

		if (xboxController.getAButton()) {
			gyroOffset = gyro.get();
			m_drivetrain.resetEncoders();
		}

		if (xboxController.getPOV() == 0) {
			gyroTurningScale += 0.01;
		} else if (xboxController.getPOV() == 180) {
			gyroTurningScale -= 0.01;
		}

		m_drivetrain.arcadeDrive(driveY, driveX);
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	public double getGyroAngle() {
	return gyro.get() - gyroOffset;
	}

	public void drive(double inches) {
		if (inches < 0) {
			if (getAvgEncoder() < (inches + 3)) {
				driveY = 0.0;

				Timer.delay(0.1);

				//speed of the robot for auto
				m_drivetrain.resetEncoders();
				autoStage++;
			} else {
				if (driveY > -0.8) {
					driveY -= 0.1;
				}
			}
		} else {
			if (getAvgEncoder() > (inches - 3)) {
				driveY = 0.0;

				Timer.delay(0.1);

				m_drivetrain.resetEncoders();
				autoStage++;
			} else {
				if (driveY < 0.8) {
					driveY += 0.1;
				}
			}
		}

		driveX = -getGyroAngle() / 15.0;

		if (driveX < -0.2) {
			driveX = -0.2;
		} else if (driveX > 0.2) {
			driveX = 0.2;
		}
	}

	public void turn(double degrees) {
		driveY = 0;

		if (degrees < 0) {
			if ((getGyroAngle() - (degrees + 30)) < 0.0) {
				driveX = 0.0;

				Timer.delay(0.1);

				//auto turn speed
				m_drivetrain.resetEncoders();
				autoStage++;
				gyroOffset += degrees;
			} else {
				driveX = -0.4;
			}
		} else {
			if ((getGyroAngle() - (degrees - 30)) > 0.0) {
				driveX = 0.0;

				Timer.delay(0.1);

				m_drivetrain.resetEncoders();
				autoStage++;
				gyroOffset += degrees;
			} else {
				driveX = 0.4;
			}
		}
	}

	public double getAvgEncoder() {
		return (leftEncoder + rightEncoder) / 2.0;
	}
}