// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {  
  private final WPI_TalonFX _talon0 = new WPI_TalonFX(Constants.drive_falcon_0);
  private final WPI_TalonFX _talon1 = new WPI_TalonFX(Constants.drive_falcon_1);
  private final WPI_TalonFX _talon2 = new WPI_TalonFX(Constants.drive_falcon_2);
  private final WPI_TalonFX _talon3 = new WPI_TalonFX(Constants.drive_falcon_3);

  private final MotorControllerGroup _leftMotor = new MotorControllerGroup(_talon0, _talon3);
  private final MotorControllerGroup _rightMotor = new MotorControllerGroup(_talon1, _talon2);

 private final TalonFXSimCollection sim_leftMotor = _talon0.getSimCollection();
private final TalonFXSimCollection sim_rightMotor = _talon1.getSimCollection();

  private final DifferentialDrive _drivetrain = new DifferentialDrive(_leftMotor, _rightMotor);
  
  private double kWheelRadiusInches = 3;
  private double kSensorGearRatio = 9.29;
  private double kCountsPerRev = 2048;
  private double k100msPerSecond = 0.1;
  
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),
    kSensorGearRatio, // Gear ratio
    10.0,  // Moment of interia kgm^2
    60.0, // Robot mass kg
    Units.inchesToMeters(kWheelRadiusInches), //Wheel radius converted to meters
    0.75, // distance between right and left wheels (m) 
    
    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  
  

  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  private ADIS16448_IMUSim m_gyroSim = new ADIS16448_IMUSim(m_gyro);
  
  private final DifferentialDriveOdometry m_odometry;
  private Field2d m_field = new Field2d();
  
  //Defining the drivetrain subsystem
  public DrivetrainSubsystem() {
    
    _talon0.configFactoryDefault();
    _talon1.configFactoryDefault();
    _talon2.configFactoryDefault();
    _talon3.configFactoryDefault();
    _leftMotor.setInverted(true);
    _talon2.setInverted(InvertType.InvertMotorOutput);

    //Defining encoders from each side of the robot.
    _talon0.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    _talon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    _talon3.follow(_talon1);
    _talon2.follow(_talon2);
    _talon3.setInverted(InvertType.FollowMaster);
    _talon2.setInverted(InvertType.FollowMaster);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
    nativeUnitsToDistanceMeters(_talon0.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition())
    );

    SmartDashboard.putData("Field", m_field);
  }


//Drivetrain Methods:

  public void drive(double leftSpeed, double rightSpeed) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    _drivetrain.tankDrive(leftSpeed, rightSpeed);

  }

  public void drive_Arcade(double speed, double rotation) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    _drivetrain.arcadeDrive(speed, rotation);

  }
  
  public void drive_Curve(double speed, double rotation, boolean turnInPlace) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    _drivetrain.curvatureDrive(speed, rotation, turnInPlace);

  }



 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and update the position and orientation of the robot.
    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
      countToDistanceMeters(_talon0.getSelectedSensorPosition()),
      countToDistanceMeters(_talon1.getSelectedSensorPosition())
    );
    m_field.setRobotPose(m_odometry.getPoseMeters());
    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    sim_leftMotor.setBusVoltage(RobotController.getBatteryVoltage());
    sim_rightMotor.setBusVoltage(RobotController.getBatteryVoltage());

    m_driveSim.setInputs(sim_leftMotor.getMotorOutputLeadVoltage(),
                        sim_rightMotor.getMotorOutputLeadVoltage());

    m_driveSim.update(0.02);

    sim_leftMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(
        m_driveSim.getLeftPositionMeters()
      ));
    sim_leftMotor.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                      m_driveSim.getLeftVelocityMetersPerSecond()
                    ));
    sim_rightMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(
        m_driveSim.getRightPositionMeters()
      ));
    sim_rightMotor.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                      m_driveSim.getRightVelocityMetersPerSecond()
                    ));
    m_gyroSim.setGyroAngleZ(m_driveSim.getHeading().getDegrees());
    
  }

 public Pose2d getPose() {
    //Method to return the current position in meters from origin
    return m_odometry.getPoseMeters();
  }
//
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
    m_odometry.resetPosition(Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
    nativeUnitsToDistanceMeters(_talon0.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition()),
    pose);
    }

  public void resetEncoders() {
    //Method to reset the encoder positions.
    _talon0.setSelectedSensorPosition(0);
    _talon1.setSelectedSensorPosition(0);
    sim_rightMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(0.0
      ));
    sim_leftMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(0.0
      ));
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
  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
}
