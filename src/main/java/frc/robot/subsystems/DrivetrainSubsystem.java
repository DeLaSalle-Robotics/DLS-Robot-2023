// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {  
  private final WPI_TalonFX _talon0 = new WPI_TalonFX(Constants.drive_falcon_0);
  private final WPI_TalonFX _talon1 = new WPI_TalonFX(Constants.drive_falcon_1);
  private final WPI_TalonFX _talon2 = new WPI_TalonFX(Constants.drive_falcon_2);
  private final WPI_TalonFX _talon3 = new WPI_TalonFX(Constants.drive_falcon_3);

  private final MotorControllerGroup _leftMotor = new MotorControllerGroup(_talon0, _talon3);
  private final MotorControllerGroup _rightMotor = new MotorControllerGroup(_talon1, _talon2);

  private final DifferentialDrive _drivetrain = new DifferentialDrive(_leftMotor, _rightMotor);
  private boolean reverse = false;

  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  private final PowerDistribution examplePD = new PowerDistribution();

  private final DifferentialDriveOdometry m_odometry;

  //Defining the drivetrain subsystem
  public DrivetrainSubsystem() {
    
    _talon0.configFactoryDefault();
    _talon1.configFactoryDefault();
    _talon2.configFactoryDefault();
    _talon3.configFactoryDefault();
    _rightMotor.setInverted(true);
    //Defining encoders from each side of the robot.
    _talon0.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    _talon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()));
  }


//Drivetrain Methods:

  public void drive(double leftSpeed, double rightSpeed) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    if (!reverse) {
      _drivetrain.tankDrive(leftSpeed, rightSpeed);
    }
    else {
      _drivetrain.tankDrive(leftSpeed, rightSpeed);
    }
  }

  public void reverse() {
    //Toggle reverse mode. Not implemented.
    reverse = !reverse;
  }

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and update the position and orientation of the robot.
    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
      countToDistanceMeters(_talon0.getSelectedSensorPosition()),
      countToDistanceMeters(_talon1.getSelectedSensorPosition())
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pose2d getPose() {
    //Method to return the current position in meters from origin
    return m_odometry.getPoseMeters();
  }

  public double getHeading() {
    //Method returns heading in degrees from original heading.
    return m_gyro.getGyroAngleZ();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //Method that returns the wheel speeds in a DifferentialDriveWheelSpeeds object. Uses a helper function to convert countPerTime to m/s
    return new DifferentialDriveWheelSpeeds(
      countPerTimeToMetersPerSecond(_talon0.getSelectedSensorVelocity()),
      countPerTimeToMetersPerSecond(_talon1.getSelectedSensorVelocity())
    );
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    //Method that sets the voltage to the motors. Used to control position and velocity. Not Implimented.
    _leftMotor.setVoltage(leftVolts);
    _rightMotor.setVoltage(rightVolts);
  }

  public void resetOdometry(Pose2d pose) {
    //Method to reset the odometry, performed as part of the intitialization protocol. Not implimented.
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()));
  }

  public void resetEncoders() {
    //Method to reset the encoder positions.
    _talon0.setSelectedSensorPosition(0);
    _talon1.setSelectedSensorPosition(0);
  }

  public double getCurrentMotorCurrent() {
    //Method to measure current going to one of the drive motors. Never implemented. 
    return examplePD.getCurrent(Constants.drive_PDP);
  }

  private double countToDistanceMeters(double sensorCounts) {
    //Method to convert encoder counts into distance in m
    double motorRotations = sensorCounts / 2048;
    double wheelRotations = motorRotations/Constants.gearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.wheelRadius));
    return positionMeters;
  }

  private double countPerTimeToMetersPerSecond(double countsPerTime) {
    //Method to convert counts per 100 ms into m/s 
    double revsPer100ms = countsPerTime / 2048;
    double wheelRevsPer100ms = revsPer100ms / Constants.gearRatio;
    double distancePer100ms = wheelRevsPer100ms * (2 * Math.PI * Units.inchesToMeters(Constants.wheelRadius));
    double distancePerSecond = distancePer100ms * 10;
    return distancePerSecond;
  }
}
