//This package is necessary to tell the computer this code belones to our robot.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN 
    public static final int drive_falcon_0 = 1; //Drivetrain Left
    public static final int drive_falcon_1 = 2; //Drivetrain Right
    public static final int drive_falcon_2 = 3; //Drivetrain Right
    public static final int drive_falcon_3 = 4; //Drivetrain Left
    public static final int armFalconRightID = 5; // Falcon driving elevator
    public static final int armFalconLeftID = 6; // Falcon driving elevator
    public static final int armExtendID = 7; // Falcon driving arm extension
    public static final int wrist775ID = 10; // ID for wrist motor
    public static int IntakeID = 0;     //Intake encoder input
    public static final int climber_snowblower = 12; //Controls climber arm position.

    //Arm Constants
    public static final int miniFalcon = 8; // ID for test arm
    
    public static final double ArmKp = 15;
    public static final double ArmKi = 0.0;
    public static final double ArmKd = 0.0;

    public static final double ArmKp_static = 15;
    public static final double ArmKi_static = 0.0;
    public static final double ArmKd_static = 0.0;

    public static final double ArmLenKp = 100.0;
    public static final double ArmLenKd = 0.0;
    public static final double ArmLenKi = 0.0;

    public static final double ArmMaxRotVel = 2.0;
    public static final double ArmMaxRotAccel = 5.0;
    
    public static final double arm_Kg = 0.54;
    public static final double arm_Ks = 0.6;
    public static final double arm_Kv = 2;
    public static final double arm_Ka = 0.05;
    
    public static final double angleTolerance = 5;
    public static final double armPositionDeg = 75.0; //Default arm position
 
    public static int drive_PDP = 1;           //Power Distribution Panal address
    
    //Robot Mechanics
    public static double gearRatio = 10.71;    //Gear ratio, needed for characterization of drivetrain
    public static double wheelRadius = 3;      //Wheel size, needed for characterization of drivetrain.
    public static double stage1Length = 24.5; //

    //Values for characterization of drivetrain <--MUST BE DETERMINED FOR OUR ROBOT!
    //Feedforward
    public static double ksVolts = 0.0761;
    public static double kvVoltsSecondsPerMeter = 2.4472;
    public static double kaVoltsSecondsSquaredPerMeter = 0.1517;
    //Feedback
    public static double kPDriveVel = 1.155; //SysID: 0.155


    //Values for characterization of wrist
    public static final double WristkSVolts = 0;
    public static final double kVWristVoltSecondsPerRotation = 0;
    public static final double WristConstants_kP = 0;
    public static final double WristConstants_kI = 0;
    public static final double WristConstants_kD = 0;
    public static final double kVVoltSecondsPerRotation = 0;

    public static final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(Units.inchesToMeters(26)); //Wheel base taken from CAD drawing.   
    
    //Sets the maximium velocities and accelerations
    public static double maxVelocityMetersPerSecond = 3;
    public static double maxAccelerationMetersPerSecondSq = 10;

    public static double ramsete_b = 2;
    public static double ramsete_z = 0.7;
   

    
    public static double Shooter_Kp = 1;
    public static double Shooter_Ki = 0;

    //Lime light related constants - useful for range finding.
    public static double limelightMountAngle = 45; // in degrees
    public static double limelightHeight = 30; // in inches
    public static double GoalHeight = 104; // in inches

    public static double ShooterSpeedSlope = 50; //Need to determine by trial and error
    public static double ShooterSpeedIntercept = 1000; //Need to determine by trial and error
    public static double kWristToleranceRPS;

    //Speed for fine control of arm.
    public static double ControlArmSpeed = 0.5;
    public static double ControlDriveSpeed = 0.5;
    
    // Balance
    public static double balanceSpeed = 0.5;
    public static double unbalanceSpeed = 0.9;

    //Intake Constants
    public static double intakeCurrentThreshold = 5;
    public static int intakeChannel = 4; //<-- The channel on the pdh inwhich the intake motors are attached.
    public static double IntakeSpeed = 0.8;
}
