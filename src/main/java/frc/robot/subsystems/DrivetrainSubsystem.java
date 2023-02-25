// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {  
  //Motor Parts
  private final WPI_TalonFX _talon1 = new WPI_TalonFX(Constants.drive_falcon_0);
  private final WPI_TalonFX _talon2 = new WPI_TalonFX(Constants.drive_falcon_1);
  private final WPI_TalonFX _talon3 = new WPI_TalonFX(Constants.drive_falcon_2);
  private final WPI_TalonFX _talon4 = new WPI_TalonFX(Constants.drive_falcon_3);
  
  private final MotorControllerGroup _leftMotor = new MotorControllerGroup(_talon1, _talon4);
  private final MotorControllerGroup _rightMotor = new MotorControllerGroup(_talon2, _talon3);

  private final DifferentialDrive _drivetrain = new DifferentialDrive(_leftMotor, _rightMotor);
  
  private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
                                                    new DifferentialDriveVoltageConstraint(
                                                        new SimpleMotorFeedforward(
                                                            Constants.ksVolts,
                                                            Constants.kvVoltsSecondsPerMeter,
                                                            Constants.kaVoltsSecondsSquaredPerMeter),
                                                        Constants.kinematics,
                                                        10);
  private final TrajectoryConfig config =
                                  new TrajectoryConfig(
                                          Constants.maxVelocityMetersPerSecond,
                                          Constants.maxAccelerationMetersPerSecondSq)
                                      // Add kinematics to ensure max speed is actually obeyed
                                      .setKinematics(Constants.kinematics)
                                      // Apply the voltage constraint
                                      .addConstraint(autoVoltageConstraint);


  private double kWheelRadiusInches = 3;
  private double kSensorGearRatio = 10.71;
  private double kCountsPerRev = 2048;
  private double k100msPerSecond = 10;
  
  
  private final double[] m_defaultVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  //Navigation
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  private final DifferentialDriveOdometry m_odometry;
  public final Field2d m_field = new Field2d();
  private AprilTagFieldLayout aprilTagFieldLayout;
  //private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

  private Pose3d m_objectInField;
  private Transform3d m_robotToCamera;
     
  //Vision Components
  private PhotonCamera llCamera;
  private Transform3d robotToCam; //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  PhotonPoseEstimator photonPoseEstimator;

  //Creating a networktable to store vision data.
  NetworkTableInstance visionTable = NetworkTableInstance.getDefault();
  NetworkTable visionData = visionTable.getTable("VisionData");
  DoubleArrayTopic cameraToObjectTopic = visionTable.getDoubleArrayTopic("VisionData");
  private final DoubleArrayEntry m_cameraToObjectEntry;
  
  


  

  //Trajectory Path
  private DifferentialDrivePoseEstimator m_poseEstimator =
  new DifferentialDrivePoseEstimator(Constants.kinematics, 
                                      new Rotation2d(this.getHeading()),
                                      this.nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition()),
                                      this.nativeUnitsToDistanceMeters(_talon2.getSelectedSensorPosition()),
                                      new Pose2d(5,5, new Rotation2d(Math.PI/2)),
                                      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), //large deviation should not trust wheels for testing vision
                                      VecBuilder.fill(1000000.5, 1000000.5, Units.degreesToRadians(30000000)));
  

  //Simulation Classes
  private final TalonFXSimCollection sim_leftMotor = _talon1.getSimCollection();
  private final TalonFXSimCollection sim_rightMotor = _talon2.getSimCollection();
  private final ADIS16448_IMUSim m_gyroSim = new ADIS16448_IMUSim(m_gyro);
  
  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2),
          kSensorGearRatio, // Gear ratio
          10.0,  // Moment of interia kgm^2
          18.1, // Robot mass kg
          Units.inchesToMeters(kWheelRadiusInches), //Wheel radius converted to meters
          0.75, // distance between right and left wheels (m) 
          
          // The standard deviations for measurement noise:
          // x and y:          0.001 m
          // heading:          0.001 rad
          // l and r velocity: 0.1   m/s
          // l and r position: 0.005 m
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  
  //Trajectory objects
  String tajectoryJSON = "paths/1_Cube_Engage.wpilib.json";
  Trajectory autoTrajectory = new Trajectory();
  
  
  //Forward Camera
  
  // Construct PhotonPoseEstimator
  

  
  

  //Defining the drivetrain subsystem
  public DrivetrainSubsystem() {
    
    SmartDashboard.putString("Trajectory Path", tajectoryJSON);
    _talon1.configFactoryDefault();
    _talon2.configFactoryDefault();
    _talon3.configFactoryDefault();
    _talon4.configFactoryDefault();
    //_leftMotor.setInverted(true);
    //_talon2.setInverted(InvertType.InvertMotorOutput);

    //Defining encoders from each side of the robot.
    _talon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    _talon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    _talon1.setNeutralMode(NeutralMode.Brake);
    _talon2.setNeutralMode(NeutralMode.Brake);
    _talon4.follow(_talon1);
    _talon3.follow(_talon2);
    _talon4.setInverted(InvertType.FollowMaster);
    _talon3.setInverted(InvertType.FollowMaster);

    /*/
    //These configurations are meant to smooth the driving by slowing acceleration a bit.
    _talon1.configNeutralDeadband(0.001);
    _talon2.configNeutralDeadband(0.001);
    _talon1.configOpenloopRamp(0.5);
    _talon2.configOpenloopRamp(0.5);
    _talon1.configClosedloopRamp(0);
    _talon1.configClosedloopRamp(0);
*/
    this.resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
    nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(_talon2.getSelectedSensorPosition())
    );
    
    //Create vision data entry - which does both publishing (.set) and subscribing (.get).

    m_cameraToObjectEntry = cameraToObjectTopic.getEntry(m_defaultVal);
    //Center of robot to camera
    m_robotToCamera = new Transform3d(new Translation3d(1, 1, 1), new Rotation3d(0, 0, Math.PI / 2));

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout("Games/aprilTagLocation.json");
     } catch (IOException ex) {
      aprilTagFieldLayout = null;
    }
//Vision Components
  llCamera = new PhotonCamera("LimeLight");
  robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
                                          new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  //photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
   //                                                       PoseStrategy.LOWEST_AMBIGUITY, llCamera, robotToCam);

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

//Camera Methods

public boolean haveTarget(){
  var result = llCamera.getLatestResult();
  return result.hasTargets();
}

public Pose3d getobjectInFieldID(){
  var result = llCamera.getLatestResult();
  int objectID;
  if (result.hasTargets()) {
    objectID = result.getBestTarget().getFiducialId();
    return aprilTagFieldLayout.getTagPose(objectID).get();
  } else {
    //For Testing always target aprilt tag 3 :
    return aprilTagFieldLayout.getTagPose(3).get();
    //return new Pose3d(curPose);
  }
  
}

public Pose2d getPoseTarget(){
  Pose2d object =  this.getobjectInFieldID().toPose2d();
  //This transform moves the center of the robot away from the vision target and faces the target.
  Transform2d robotToTarget = new Transform2d(new Translation2d(0.75, 0.0), new Rotation2d(Math.PI));
  return object.transformBy(robotToTarget);
}

public Trajectory targetTrajectory() {
  //Has issues if curPose.x is lower than tarPose.x, the trajectory moves past target.
  Pose2d curPose = m_odometry.getPoseMeters();
  Pose2d tarPose = this.getPoseTarget();
  var interiorWaypoint = new ArrayList<Translation2d>();
  interiorWaypoint.add(new Translation2d((tarPose.getX() - curPose.getX())/2 + curPose.getX(),tarPose.getY() ));
 
  Trajectory traj = TrajectoryGenerator.generateTrajectory(
    curPose, interiorWaypoint, tarPose, config);
    m_field.getObject("Target").setTrajectory(traj);
    
    System.out.println("Sent new traj");
    return traj;
}


public Transform3d getCameraToObject() {
  var result = llCamera.getLatestResult();
  Transform3d cameraToObjectTransform;
  if (result.hasTargets()) {
    cameraToObjectTransform = result.getBestTarget().getBestCameraToTarget();
  } else {cameraToObjectTransform = new Transform3d(new Pose3d(this.getPose()),
                                                      this.getobjectInFieldID());}
                                                    
  return cameraToObjectTransform;
}

 
public Pose3d objectToRobotPose(
  Pose3d objectInField, Transform3d robotToCamera, Transform3d cameraToObject) {
    
    return ComputerVisionUtil.objectToRobotPose(objectInField, cameraToObject, robotToCamera);
  }

  /*
  public void publishCameraToObject(
    Pose3d objectInField,
    Transform3d robotToCamera,
    DoubleArrayEntry cameraToObjectEntry,
    DifferentialDrivetrainSim m_driveSim) {
      Pose3d robotInField = new Pose3d(m_driveSim.getPose());
      Pose3d cameraInField = robotInField.plus(robotToCamera);
      Transform3d cameraToObject= new Transform3d(cameraInField, objectInField);
      // Publishes double array with Translation3D elements {x, y, z} and Rotation3D elements {w, x,
      // y, z} which describe
      // the cameraToObject transformation.
      double[] val = {
        cameraToObject.getX(),
        cameraToObject.getY(),
        cameraToObject.getZ(),
        cameraToObject.getRotation().getQuaternion().getW(),
        cameraToObject.getRotation().getQuaternion().getX(),
        cameraToObject.getRotation().getQuaternion().getY(),
        cameraToObject.getRotation().getQuaternion().getZ()
      };
      cameraToObjectEntry.set(val);
    }
    */
   

public void updateOdometry() {
  /*m_poseEstimator.update(new Rotation2d(-m_gyro.getAngle()), 
                          countToDistanceMeters(_talon1.getSelectedSensorPosition()),
                          countToDistanceMeters(_talon2.getSelectedSensorPosition()));
  */
    // Publish cameraToObject transformation to networktables --this would normally be handled by
    // the
    // computer vision solution.
    //publishCameraToObject(
    //    m_objectInField, m_robotToCamera, m_cameraToObjectEntry, m_driveSim);
    
    // Compute the robot's field-relative position exclusively from vision measurements.
    /*
    Pose3d visionMeasurement3d = 
      objectToRobotPose(this.getobjectInFieldID(this.getPose()), m_robotToCamera, this.getCameraToObject());

    // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.

    Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

  // Apply vision measurements. For simulation purposes only, we don't input a latency delay -- on
    // a real robot, this must be calculated based either on known latency or timestamps.
   // m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
   */
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and update the position and orientation of the robot.
    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
      countToDistanceMeters(_talon1.getSelectedSensorPosition()),
      countToDistanceMeters(_talon2.getSelectedSensorPosition())
    );
    m_field.setRobotPose(m_odometry.getPoseMeters());
    
    SmartDashboard.putNumber("Left Speed", this.getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed", this.getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Left EncoderVel", _talon1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right EncoderVel", _talon2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Heading", this.getHeading());

    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    sim_leftMotor.setBusVoltage(RobotController.getBatteryVoltage());
    sim_rightMotor.setBusVoltage(RobotController.getBatteryVoltage());

    m_driveSim.setInputs(sim_leftMotor.getMotorOutputLeadVoltage(),
                        sim_rightMotor.getMotorOutputLeadVoltage());
    SmartDashboard.putNumber("LeftSimMotor", sim_leftMotor.getMotorOutputLeadVoltage());
    SmartDashboard.putNumber("RightSimMotor", sim_rightMotor.getMotorOutputLeadVoltage());

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
    m_gyroSim.setGyroAngleZ(-m_driveSim.getHeading().getDegrees());
    
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

  public double getTurnRate() {
    return m_gyro.getRate();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //Method that returns the wheel speeds in a DifferentialDriveWheelSpeeds object. Uses a helper function to convert countPerTime to m/s
    return new DifferentialDriveWheelSpeeds(
      countPerTimeToMetersPerSecond(_talon1.getSelectedSensorVelocity()),
      countPerTimeToMetersPerSecond(_talon2.getSelectedSensorVelocity())
    );
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    //Method that sets the voltage to the motors. Used to control position and velocity. Not Implimented.
    _leftMotor.setVoltage(leftVolts);
    _rightMotor.setVoltage(rightVolts);
    _drivetrain.feed();
    SmartDashboard.putNumber("Left Volt", leftVolts);
  }

  public void resetOdometry(Pose2d pose) {
    //Method to reset the odometry, performed as part of the intitialization protocol. Not implimented.
    this.resetEncoders();

    //m_gyro.reset();
    m_odometry.resetPosition(Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()),
    nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(_talon2.getSelectedSensorPosition()),
    pose);
    }

  public void resetEncoders() {
    //Method to reset the encoder positions.
    _talon1.setSelectedSensorPosition(0);
    _talon2.setSelectedSensorPosition(0);
    sim_rightMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(0.0
      ));
    sim_leftMotor.setIntegratedSensorRawPosition(
      distanceToNativeUnits(0.0
      ));
      System.out.println("Resetting Encoders");
  }

  public double getAverageEncoderDistance() {
    return (this.countToDistanceMeters(_talon1.getSelectedSensorPosition()) + this.countToDistanceMeters(_talon2.getSelectedSensorPosition())) / 2;
  }

public void setMaxOutput(double maxOutput){
  _drivetrain.setMaxOutput(maxOutput);
}

public void setNewPose(Pose2d pose){
  this.resetEncoders();
  m_odometry.resetPosition(new Rotation2d(0),0.0, 0.0, pose);
  m_field.setRobotPose(pose);
  
}

public void zeroHeading() {
  m_gyro.reset();
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


public Trajectory getTrajectory() {
  if (DriverStation.isTeleop()) {
    return this.targetTrajectory();
  } else {
  try {
    String trajectoryJSON = SmartDashboard.getString("Trajectory Path", tajectoryJSON);
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    if (Files.exists(trajectoryPath)){System.out.println("Trajectory Exists");}
    Trajectory autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  
  System.out.println("Trajectory Read");
  m_field.getObject("Traj").setTrajectory(autoTrajectory);
  return autoTrajectory;
 } catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());    
  Trajectory autoTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(5, 5, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(6, 4), new Translation2d(7, 6)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(8, 5, new Rotation2d(0)),
      // Pass config
      config);
    System.out.println("Trajectory Created");
    m_field.getObject("Traj").setTrajectory(autoTrajectory);
    return autoTrajectory;
 }
 }
}

public void clearTrajectories(){
  Trajectory nullTrajectory = 
    TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), 
    List.of(new Translation2d(0.1, 0), new Translation2d(0, 0)),
    new Pose2d(0,0,new Rotation2d(0)), config);
    m_field.getObject("Traj").setTrajectory(nullTrajectory);
    
}

public Trajectory getTrajectoryPath(String trajectoryJSON) {

  try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  if (Files.exists(trajectoryPath)){System.out.println("Trajectory Exists");}
  Trajectory autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  return autoTrajectory;
 } catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  Trajectory autoTrajectory = this.getTrajectory();
  return autoTrajectory;
 }
}

}

