// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.List;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBot_DrivetrainSubsystem extends SubsystemBase {  
  private final WPI_VictorSPX Cim0 = new WPI_VictorSPX(Constants.drive_cim_0);
  private final WPI_VictorSPX Cim2 = new WPI_VictorSPX(Constants.drive_cim_2);
  private final WPI_VictorSPX Cim3 = new WPI_VictorSPX(Constants.drive_cim_3);
  private final WPI_VictorSPX Cim4 = new WPI_VictorSPX(Constants.drive_cim_4);

  private final MotorControllerGroup _leftMotor = new MotorControllerGroup(Cim0, Cim2);
  private final MotorControllerGroup _rightMotor = new MotorControllerGroup(Cim3, Cim4);

//private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();
 PhotonCamera camera = new PhotonCamera("Cube_cam");
 final double ANGULAR_P = 0.03;
 final double ANGULAR_D = 0.0;
 PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
 final double LINEAR_P = 0.03;
 final double LINEAR_D = 0.0;
 PIDController ForwardController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
 
  //Defining the drivetrain subsystem
  public DriveBot_DrivetrainSubsystem() {
  _rightMotor.setInverted(true);
   }
  private final DifferentialDrive _drivetrain = new DifferentialDrive(_leftMotor, _rightMotor);

//Drivetrain Methods:

  public void drive(double leftSpeed, double rightSpeed) {
    //Sends speeds to tank drive. Considered implementing a reverse driving capacity, but eventually abandoned that.
    // Note: tankdrive is a method of the DifferentialDrive class.
    _drivetrain.tankDrive(leftSpeed, rightSpeed);
// left side = stick 
// right side = -stick

  }

  public void drive_Arcade(double speed, double rotation) {
    
    _drivetrain.arcadeDrive(speed, rotation);

  }

 

  public void find_cube(){
    
    var result = camera.getLatestResult();

    double rotationSpeed;
    double forwardSpeed;
    if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        forwardSpeed = turnController.calculate(result.getBestTarget().getArea(), 15);
        rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
        if (rotationSpeed > 0.5){rotationSpeed = 0.5;}
    } else {
        // If we have no targets, spin slowly.
        forwardSpeed = 0;
        rotationSpeed = 0.2;
    }
    SmartDashboard.putNumber("Cube Rotation", rotationSpeed);
// Use our forward/turn speeds to control the drivetrain
this.drive_Arcade(forwardSpeed, rotationSpeed );
  }




  
/* 
  public double getHeading() {
    //Method returns heading in degrees from original heading.
    return m_gyro.getGyroAngleZ();
  }
  

public void zeroHeading() {
  m_gyro.reset();
}
*/
}

