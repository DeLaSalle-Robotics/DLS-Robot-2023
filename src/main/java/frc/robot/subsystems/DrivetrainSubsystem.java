// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Unused imports
//import java.lang.reflect.Array;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.hal.SimDevice;
//import edu.wpi.first.math.ComputerVisionUtil;
//import edu.wpi.first.math.MatBuilder;
//import edu.wpi.first.math.Nat;
//import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
//import edu.wpi.first.math.geometry.Quaternion;
//import edu.wpi.first.networktables.BooleanTopic;
//import edu.wpi.first.networktables.DoubleArrayEntry;
//import edu.wpi.first.networktables.DoubleArrayPublisher;
//import edu.wpi.first.networktables.DoubleArraySubscriber;
//import edu.wpi.first.networktables.DoubleArrayTopic;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.Topic;
//import edu.wpi.first.wpilibj.ADIS16448_IMU;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
//import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

import frc.robot.Constants;
import frc.robot.Robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {  
  //Motor Parts
  private final WPI_TalonFX _talon1 = new WPI_TalonFX(Constants.drive_falcon_1);
  private final WPI_TalonFX _talon2 = new WPI_TalonFX(Constants.drive_falcon_2);
  private final WPI_TalonFX _talon3 = new WPI_TalonFX(Constants.drive_falcon_3);
  private final WPI_TalonFX _talon4 = new WPI_TalonFX(Constants.drive_falcon_4);
  //Motor Groups
  private final MotorControllerGroup _leftMotor = new MotorControllerGroup(_talon1, _talon4);
  private final MotorControllerGroup _rightMotor = new MotorControllerGroup(_talon2, _talon3);
  //Defining DifferentialDrive object
  private final DifferentialDrive _drivetrain = new DifferentialDrive(_leftMotor, _rightMotor);
  //Placing voltage constraint to allow trajectory following. Constants determined by SysID
  private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
                                                    new DifferentialDriveVoltageConstraint(
                                                        new SimpleMotorFeedforward(
                                                            Constants.ksVolts,
                                                            Constants.kvVoltsSecondsPerMeter,
                                                            Constants.kaVoltsSecondsSquaredPerMeter),
                                                        Constants.kinematics,
                                                        10);

  //Trajectory config sets max velocity and acceleration during trajectory following.
  private final TrajectoryConfig config =
                                  new TrajectoryConfig(
                                          Constants.maxVelocityMetersPerSecond,
                                          Constants.maxAccelerationMetersPerSecondSq)
                                      // Add kinematics to ensure max speed is actually obeyed
                                      .setKinematics(Constants.kinematics)
                                      // Apply the voltage constraint
                                      .addConstraint(autoVoltageConstraint);


//These could probably go in the constants class
  private double kWheelRadiusInches = 3;
  private double kSensorGearRatio = 10.71;
  private double kCountsPerRev = 2048;
  private double k100msPerSecond = 10;
  
  
  private final double[] m_defaultVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  //Navigation <- Creating the NavX object
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private double initialHeading;
  //Creating the odometry object to allow tracking of our robot
  private final DifferentialDriveOdometry m_odometry;
  //Creating a field, primarily for testing
  public final Field2d m_field = new Field2d();
  //Creating an object that will hold the aprilTag locations
  private AprilTagFieldLayout aprilTagFieldLayout;

  private Pose3d m_objectInField;
  
  //These objects will hold the distance of the camera from the robot center -- may not need - should be in Constants class ?
  private Transform3d m_robotToLimeLight;
  private Transform3d m_robotToCubeCame;
     
  //Vision Components
  private PhotonCamera llCamera;
  private PhotonCamera cubeCam;

  //PID controllers that find the cube
  final double ANGULAR_P = 1; //<-- should be moved to Constants class to allow it to be found.
  PIDController turnController = new PIDController(ANGULAR_P, 0, 0);
  final double LINEAR_P = 1; //<-- should be moved to Constants class to allow it to be found.
  PIDController forwardController = new PIDController(LINEAR_P, 0, 0);
  
  //Simulation Classes - Motor groups and gyro
  private final TalonFXSimCollection sim_leftMotor = _talon1.getSimCollection();
  private final TalonFXSimCollection sim_rightMotor = _talon2.getSimCollection();
  
  int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  
  
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
  
 //Auto balance code:

private int state;
private int debounceCount;
private double robotSpeedSlow;
private double robotSpeedFast;
private double onChargeStationDegree;
private double levelDegree;
private double debounceTime;
private double singleTapTime;
private double scoringBackUpTime;
private double doubleTapTime;

  //Defining the drivetrain subsystem
  public DrivetrainSubsystem() {
    
    _talon1.configFactoryDefault();
    _talon2.configFactoryDefault();
    _talon3.configFactoryDefault();
    _talon4.configFactoryDefault();
    if (Constants.limitFalcons){
    //Current limiting drivetrain motors to limit brownout potential in a pushing match.
    _talon1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
    _talon2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
    _talon3.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
    _talon4.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
      }    
    //Defining encoders from each side of the robot.
    _talon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    _talon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    //Defining Brake mode
    _talon1.setNeutralMode(NeutralMode.Brake);
    _talon2.setNeutralMode(NeutralMode.Brake);
    

    //Setting up motor directions
    _talon4.follow(_talon1);
    _talon3.follow(_talon2);
    _talon4.setInverted(InvertType.FollowMaster);
    _talon3.setInverted(InvertType.FollowMaster);

   
    //These configurations are meant to smooth the driving by slowing acceleration/decelleration a bit. Hopefully preventing tipping
    _talon1.configNeutralDeadband(0.001);
    _talon2.configNeutralDeadband(0.001);
    _talon1.configOpenloopRamp(0.5);
    _talon2.configOpenloopRamp(0.5);
    _talon1.configClosedloopRamp(0);
    _talon1.configClosedloopRamp(0);

    
    this.resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_gyro.getYaw()),
    nativeUnitsToDistanceMeters(_talon1.getSelectedSensorPosition()),
    nativeUnitsToDistanceMeters(_talon2.getSelectedSensorPosition())
    );
    
    SmartDashboard.putNumber("Initial Heading", m_gyro.getYaw());
    SmartDashboard.putBoolean("Set Heading",true);

    //Loading the AprilTag Locations from JSON
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout("Games/aprilTagLocation.json");
     } catch (IOException ex) {
      aprilTagFieldLayout = null;
    }
//Vision Components
  llCamera = new PhotonCamera("LimeLight");
  m_robotToLimeLight = new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
                                          new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  cubeCam = new PhotonCamera("Cube_cam"); // Offset calculated in photonvision work flow.

    SmartDashboard.putData("Field", m_field);

    //Balance Code
    state = 0;
    debounceCount = 0;
    
    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.4;
    
    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.4;
    
    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree =  13.0;
    
    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0

    levelDegree = 6.0;
    
    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.2;
    
    // Amount of time to drive towards to scoring target when trying to bump the
    // game piece off
    // Time it takes to go from starting position to hit the scoring target
    singleTapTime = 0.4;
    
    // Amount of time to drive away from knocked over gamepiece before the second
    // tap
    scoringBackUpTime = 0.2;
    
    // Amount of time to drive forward to secure the scoring of the gamepiece
    doubleTapTime = 0.3;
  }

public void resetGyro() {
  m_gyro.reset();
}
//Drivetrain Methods:

  public void drive_Arcade(double speed, double rotation) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    _drivetrain.arcadeDrive(speed, rotation);

  }
  
//Camera Methods

public boolean have_target(){
  var result = cubeCam.getLatestResult();
  if (result.hasTargets()){
    SmartDashboard.putBoolean("Cube Target", true);
  } else {
    SmartDashboard.putBoolean("Cube Target", true);
  }
  return result.hasTargets();
}

//This method will return forward and rotation values- will need to be called by cube gathering command
public double[] find_cube(){
    
  var result = cubeCam.getLatestResult();
  double rotationSpeed;
  double forwardSpeed;
  if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      forwardSpeed = forwardController.calculate(result.getBestTarget().getArea(), 15); //<-- The target area should be in the Constants class.
      rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0); // <-- Target Yaw will need to be updated once camera and intake are mounted. Also should be in Constants
      if (rotationSpeed > 0.5){rotationSpeed = 0.5;} // limit rotation speed
  } else {
      forwardSpeed = 0;
      rotationSpeed = 0;
  }
// Use our forward/turn speeds to control the drivetrain
double[] array = {forwardSpeed, rotationSpeed};
return array;
}

public double targetRotation(double target) {
  double rotationSpeed = turnController.calculate(this.getHeading(),target);
  return rotationSpeed;
}

//This method returns if AT Target is acquired
public boolean haveATTarget(){
  var result = llCamera.getLatestResult();
  if (result.hasTargets()) {
    SmartDashboard.putBoolean("ATTarget", true);
} else {
  SmartDashboard.putBoolean("ATTarget", false);}
  return result.hasTargets();
}

//This method returns the position of the AT target, if none returns current position.
public Pose3d getobjectInFieldID(){
  var result = llCamera.getLatestResult();
  int objectID;
  if (result.hasTargets()) {
    objectID = result.getBestTarget().getFiducialId();
    return aprilTagFieldLayout.getTagPose(objectID).get();
  } else {
    //For Testing always target aprilt tag 2 :
    if (Robot.isSimulation()) {return aprilTagFieldLayout.getTagPose(2).get();
    } else {
    return new Pose3d(m_odometry.getPoseMeters());}
  }
  
}

public Pose2d visionPose(){
  Pose3d tarPose = this.getobjectInFieldID();
  var result = llCamera.getLatestResult();
  if (result.hasTargets()) {
    Pose2d newPose = tarPose.plus(result.getBestTarget().getBestCameraToTarget()).toPose2d();
    return newPose;
    } else {
      return m_odometry.getPoseMeters();
    }

}
//Returns a pose that is slightly removed from the target in the x, aligned in the y, and oppisite in the rotation.
public Pose2d getPoseTarget(){
  Pose2d object =  this.getobjectInFieldID().toPose2d();
  //This transform moves the center of the robot away from the vision target and faces the target.
  Transform2d robotToTarget = new Transform2d(new Translation2d(0.75, 0.0), new Rotation2d(Math.PI));
  return object.transformBy(robotToTarget);
}

//Creates the trajectory needed to get to the target pose.
// public Trajectory targetTrajectoryold() {
//   //Has issues if curPose.x is lower than tarPose.x, the trajectory moves past target.
//   Pose2d curPose = m_odometry.getPoseMeters();
//   Pose2d tarCenterPose = this.getPoseTarget();
//   double tarRot = tarCenterPose.getRotation().getDegrees();
//   SmartDashboard.putNumber("Tar Degree", tarRot);
//   Pose2d tarPose = new Pose2d(new Translation2d(tarCenterPose.getX() ,
//                                       tarCenterPose.getY()+ SmartDashboard.getNumber("Shift", 0)),
//                                       new Rotation2d(0.0));
//   var interiorWaypoint = new ArrayList<Translation2d>();
//   interiorWaypoint.add(new Translation2d((tarPose.getX() - curPose.getX())/2 + curPose.getX(),tarPose.getY() ));
 
//   Trajectory traj = TrajectoryGenerator.generateTrajectory(
//     curPose, interiorWaypoint, tarPose, config);
//     m_field.getObject("Target").setTrajectory(traj);
    
//     System.out.println("Sent new traj");
//     return traj;
// }

public Trajectory targetTrajectory() {
  //Has issues if curPose.x is lower than tarPose.x, the trajectory moves past target.
  Pose2d curPose = m_odometry.getPoseMeters();
  Pose2d tarCenterPose = this.getPoseTarget();
  double tarRot = tarCenterPose.getRotation().getDegrees();
  double curRot = curPose.getRotation().getDegrees();
  SmartDashboard.putNumber("Tar Degree", tarRot);
  SmartDashboard.putNumber("Cur Degree", curRot);
  double placementOffset;
  //Changes the direction of offset dependent on alliance.
  if (SmartDashboard.getBoolean("Alliance", false)) {
    placementOffset = -0.5;
  } else {
    placementOffset = 0.5;
  }
  Pose2d tarPose = new Pose2d(new Translation2d(tarCenterPose.getX() + placementOffset,
                                      tarCenterPose.getY()+ SmartDashboard.getNumber("Shift", 0)),
                                      new Rotation2d(0.0));
  double needRot = Math.atan((curPose.getY() - tarPose.getY())/(curPose.getX() - tarPose.getX()));
  if (Constants.verbose) {SmartDashboard.putNumber("Need Rot", Math.toDegrees(needRot));}
  Pose2d startPose = new Pose2d(new Translation2d(curPose.getX(),curPose.getY()), new Rotation2d(needRot));
  Pose2d endPose = new Pose2d(new Translation2d(tarPose.getX(),tarPose.getY()), new Rotation2d(needRot));

  var interiorWaypoint = new ArrayList<Translation2d>();
  interiorWaypoint.add(new Translation2d((endPose.getX() - startPose.getX())/2 + startPose.getX(),
  (endPose.getY() - startPose.getY())/2 + startPose.getY() ));
  
  Trajectory traj = TrajectoryGenerator.generateTrajectory(
    startPose, interiorWaypoint, endPose, config);
    m_field.getObject("Target").setTrajectory(traj);
    return traj;
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run and update the position and orientation of the robot.
    //Updating the odometry each tick allows is to keep track of where we are.
    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getYaw()),
      countToDistanceMeters(_talon1.getSelectedSensorPosition()),
      countToDistanceMeters(_talon2.getSelectedSensorPosition())
    );
    //Putting the robot on the field helps with planning.
    // SmartDashboard.putBoolean("April Target", this.haveATTarget());
    // SmartDashboard.putBoolean("Cube Target", this.have_target());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    boolean scoreFront;
    boolean loadFront;
    double curHeading = this.getHeading();
    if (Constants.verbose) {SmartDashboard.putNumber("Robot Heading", this.getHeading());}
    if (Math.abs(curHeading) > 180){
      curHeading = curHeading % 360;
    }
    if (Math.abs(curHeading) < 45){
      scoreFront = true;
    } else if (Math.abs(curHeading) > 135) {
      scoreFront = false;
    } else { scoreFront = false;}
    SmartDashboard.putBoolean("Score Front", scoreFront);
    if (SmartDashboard.getBoolean("Red Alliance", true)) {
       if (curHeading <= 135 && curHeading >= 45) {
        loadFront = false;
      } else if (curHeading >= -135 && curHeading <= -45){
        loadFront = true;
      } else {loadFront = false;} 
    } else {
      //If Blue Alliance - everything is flipped
      if (curHeading <= 135 && curHeading >= 45) {
        loadFront = true;
      } else if (curHeading >= -135 && curHeading <= -45){
        loadFront = false ;
      } else {loadFront = false;} 
    }
    SmartDashboard.putBoolean("Load Front", loadFront);
    SmartDashboard.putNumber("Pitch:",this.getPitch());
    SmartDashboard.putNumber("Roll:",this.getRoll());
    SmartDashboard.putNumber("Case:",state);
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

    //Simulating the NavX
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    gyroSimAngle.set(-m_driveSim.getHeading().getDegrees());
    
  }

 public Pose2d getPose() {
    //Method to return the current position in meters from origin
    return m_odometry.getPoseMeters();
  }
//
  public double getHeading() {
    SmartDashboard.putNumber("Pitch_", m_gyro.getPitch());
    SmartDashboard.putNumber("Roll_", m_gyro.getRoll());
    SmartDashboard.putNumber("Yaw_", m_gyro.getYaw());
    //Method returns heading in degrees from original heading.
    if (SmartDashboard.getBoolean("Set Heading", false)) {
      SmartDashboard.putNumber("Initial Heading", m_gyro.getYaw());
      SmartDashboard.putBoolean("Set Heading",false);
    }
    initialHeading = SmartDashboard.getNumber("Initial Heading", 0);
    return m_gyro.getYaw() - initialHeading;
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
    //Method that sets the voltage to the motors. Used to control position and velocity. 
    _leftMotor.setVoltage(leftVolts);
    _rightMotor.setVoltage(rightVolts);
    _drivetrain.feed();
  }

  public void resetOdometry(Pose2d pose) {
    //Method to reset the odometry, performed as part of the intitialization protocol.
    this.resetEncoders();

    //m_gyro.reset();
    m_odometry.resetPosition(Rotation2d.fromDegrees(-m_gyro.getYaw()),
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
  }

  //Returns the average distance the robot has moved.
  public double getAverageEncoderDistance() {
    return (this.countToDistanceMeters(_talon1.getSelectedSensorPosition()) + this.countToDistanceMeters(_talon2.getSelectedSensorPosition())) / 2;
  }


public void zeroHeading() {
  m_gyro.reset();
}

//Converts encoder counts into meters
  private double countToDistanceMeters(double sensorCounts) {
    //Method to convert encoder counts into distance in m
    double motorRotations = sensorCounts / 2048;
    double wheelRotations = motorRotations/Constants.gearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.wheelRadius));
    return positionMeters;
  }
// Converts encoder rate to m/s velocity
  private double countPerTimeToMetersPerSecond(double countsPerTime) {
    //Method to convert counts per 100 ms into m/s 
    double revsPer100ms = countsPerTime / 2048;
    double wheelRevsPer100ms = revsPer100ms / Constants.gearRatio;
    double distancePer100ms = wheelRevsPer100ms * (2 * Math.PI * Units.inchesToMeters(Constants.wheelRadius));
    double distancePerSecond = distancePer100ms * 10;
    return distancePerSecond;
  }
  //Converts a distance into encoder counts
  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  //Converts velocity (m/s) into encoder rates
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

//This method provides a means of exporting tragetories - primarily used for targetTrajectory during teleop
//Autonomous commands will call the trajectories themselves.
public Trajectory getTrajectory() {
  if (DriverStation.isTeleop()) {
    return this.targetTrajectory();
  } else {
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
    m_field.getObject("Traj").setTrajectory(autoTrajectory);
    return autoTrajectory;
 }
 }


// //Erases Trajectories on the simulated field
public void clearTrajectories(){
  Trajectory nullTrajectory = 
    TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), 
    List.of(new Translation2d(0.1, 0), new Translation2d(0, 0)),
    new Pose2d(0,0,new Rotation2d(0)), config);
    m_field.getObject("Traj").setTrajectory(nullTrajectory);
    
}

// Returns a Trajectory object when given a trajectory path
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

public Trajectory refTraj(){
  Trajectory autoTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(5, 5, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(6.5, 5),
       new Translation2d(8, 6),
       new Translation2d(6.5,5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(5, 5, new Rotation2d(0)),
      // Pass config
      config);
      return autoTrajectory;
}

public double getPitch(){
  return m_gyro.getPitch();
}
public double getRoll(){
  return m_gyro.getRoll();
}
public void tipProtection(){
  //Method that will stop the robot if roll angle/accelleration gets to large
  // Protection during autonoums in case we get too close to the charging station - ?
}

public void postTrajectories (Trajectory traj){
  m_field.getObject("Traj").setTrajectory(traj);
}

public int secondsToTicks(double time) {
  return (int) (time / 0.02);
}




public double autoBalanceRoutine(double init) {
  switch (state) {
      // drive forwards to approach station, exit when tilt is detected
      case 0:
          if (Math.abs(this.getRoll()) > onChargeStationDegree) {
              debounceCount++;
          }
          if (debounceCount > this.secondsToTicks(debounceTime)) {
              state = 1;
              debounceCount = 0;
              return robotSpeedSlow;
          }
          return robotSpeedFast;
      // driving up charge station, drive slower, stopping when level
      case 1:
          if (this.getRoll() < levelDegree) {
              debounceCount++;
          }
          if (debounceCount > this.secondsToTicks(debounceTime)) {
              state = 2;
              debounceCount = 0;
              return 0;
          }
          return robotSpeedSlow;
      // on charge station, stop motors and wait for end of auto
      case 2:
          if (Math.abs(this.getRoll()-init) <= Math.abs(levelDegree / 2)) {
              debounceCount++;
          }
          if (debounceCount > this.secondsToTicks(debounceTime)) {
              state = 4;
              debounceCount = 0;
              return 0;
          }
          if (this.getRoll()-init >= levelDegree) {
              debounceCount = 0;
              return -1 * robotSpeedSlow;
          } else if (this.getRoll()-init <= -levelDegree) {
              debounceCount = 0;
              return robotSpeedSlow;
          }
      case 3:
          return 0;
  }
  return 0;
}
}

