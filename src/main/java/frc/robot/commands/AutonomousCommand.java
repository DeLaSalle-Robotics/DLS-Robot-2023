package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class AutonomousCommand extends CommandBase {
    private DrivetrainSubsystem m_drive;

    DifferentialDriveVoltageConstraint autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.ksVoltsSecondsPerMeter,
            Constants.kVoltsSecondsSquaredPerMeter), 
            Constants.kinematics, 
            10); //Max voltage

    TrajectoryConfig config = 
    new TrajectoryConfig(Constants.maxVelocityMetersPerSecond, 
    Constants.maxAccelerationMetersPerSecondSq);

    Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)), //Starting point
        // forward 1 meter and over 1 meter, then forward another and back to the same line
        List.of(new Translation2d(1,1), new Translation2d(2,-1)) , 
        // final pose is 3 meters forward
         new Pose2d(3,0, new Rotation2d(0)), config);
/*
         //This command can be used tp import Pathweaver tragectories.
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
*/
    RamseteCommand ramseteCommand = 
    new RamseteCommand(
        //exampleTrajectory,
        exampleTrajectory,
         m_drive::getPose,
         new RamseteController(Constants.ramsete_b, Constants.ramsete_z), 
         new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.ksVoltsSecondsPerMeter,
            Constants.kVoltsSecondsSquaredPerMeter),
            Constants.kinematics, 
        m_drive::getWheelSpeeds, 
        new PIDController(Constants.kPDrive, 0, 0), 
        new PIDController(Constants.kPDrive, 0, 0),
        m_drive::driveVolts, m_drive);
    
        @Override
        public void end(boolean interrupted) {
      m_drive.resetOdometry(exampleTrajectory.getInitialPose());
      m_drive.drive(0, 0);
        }

        
}
