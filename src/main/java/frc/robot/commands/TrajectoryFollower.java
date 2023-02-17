package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollower extends CommandBase{
    private Trajectory m_trajectory;
    private DrivetrainSubsystem m_driveSubsystem;
    private double m_prevTime = -1 ;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private RamseteController m_follower = new RamseteController(Constants.ramsete_b, Constants.ramsete_z);;
    private final Timer m_timer = new Timer();
    private BiConsumer<Double, Double> m_output;
    private Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private Supplier<Pose2d> m_pose;
    private Pose2d initpose;
    private DifferentialDriveKinematics m_kinematics= Constants.kinematics;
    private SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(Constants.ksVolts,
                                                                            Constants.kvVoltsSecondsPerMeter,
                                                                            Constants.kaVoltsSecondsSquaredPerMeter);
    private PIDController m_leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    private PIDController m_rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    
    public TrajectoryFollower(Trajectory _trajectory, Pose2d _initpose,DrivetrainSubsystem _driveSubsystem) {
        this.m_driveSubsystem = _driveSubsystem;
        this.m_trajectory = _trajectory;
        this.initpose = _initpose;
        this.m_output = m_driveSubsystem::driveVolts;
        this.m_speeds = m_driveSubsystem::getWheelSpeeds;
        this.m_pose = m_driveSubsystem::getPose;
    
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveSubsystem);}
        
        
    @Override
    public void initialize() {
        /*Creating the controller each time the command is called. */
        
        System.out.println("Initializing Command");
        
        /* Setting speeds and controllers to zero*/
        m_prevTime = -1;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = 
            m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(
                    initialState.velocityMetersPerSecond,
                    0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond)
                );
        m_timer.reset();
        m_timer.start();
        m_leftController.reset();
        m_rightController.reset();
        m_driveSubsystem.resetOdometry(initpose);
    }
    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;
        SmartDashboard.putNumber("Time", curTime);
        if(m_prevTime < 0) {
            m_output.accept(0.0,0.0);
            m_prevTime = curTime;
            return;
        }

        var targetWheelSpeeds =
            m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(),m_trajectory.sample(curTime)));
        
        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        SmartDashboard.putNumber("Left Speed Target", leftSpeedSetpoint);
        SmartDashboard.putNumber("Right Speed Target", rightSpeedSetpoint);
       

        double leftOutput;
        double rightOutput;

        double leftFeedforward = 
                m_Feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond)/ dt);
        double rightFeedforward = 
                m_Feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond)/ dt);

        leftOutput = 
            leftFeedforward 
                + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

        rightOutput = 
            rightFeedforward 
                + m_rightController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
        m_output.accept(leftOutput, rightOutput);
        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Left Feed Forward", leftFeedforward);
        SmartDashboard.putNumber("Left PID", m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint));
        
        
        SmartDashboard.putNumber("Right Output", rightOutput);
        SmartDashboard.putNumber("Right Feed Forward", rightFeedforward);
        SmartDashboard.putNumber("Right PID", m_leftController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint));
        
        SmartDashboard.putNumber("Distance", m_driveSubsystem.getAverageEncoderDistance());
    }
    @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_leftController.close();
    m_rightController.close();
    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
