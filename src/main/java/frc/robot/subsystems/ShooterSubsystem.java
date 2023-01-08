// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DigitalIOSwitch;

public class ShooterSubsystem extends SubsystemBase {  
  private final WPI_TalonFX _falconA = new WPI_TalonFX(Constants.shooter_falcon0);
  private final WPI_TalonFX _falconB = new WPI_TalonFX(Constants.shooter_falcon1);
  private final VictorSPX _snowblower = new VictorSPX(Constants.shooter_snowblower);
  private DigitalIOSwitch hood_lowerswitch = new DigitalIOSwitch(Constants.shooter_lowerswitchchannel);
  private DigitalIOSwitch hood_upperswitch = new DigitalIOSwitch(Constants.shooter_upperswitchchannel);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
//Declaring the Shooter Subsystem and configuring motors \/
public ShooterSubsystem() {
  _falconA.configFactoryDefault();
  _falconB.configFactoryDefault();
  _falconB.follow(_falconA);
  _falconA.setInverted(true);
  _falconB.setInverted(false);
  _falconA.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.shooterTimeOut);
  //Attempting to simplify code, this should allow is to only 
  //change inputs to _falconA and have _falconB do the same, but inverted.
/* Config the peak and nominal outputs */
_falconA.configNominalOutputForward(0, Constants.shooterTimeOut);
_falconA.configNominalOutputReverse(0, Constants.shooterTimeOut);
_falconA.configPeakOutputForward(1, Constants.shooterTimeOut);
_falconA.configPeakOutputReverse(-1, Constants.shooterTimeOut);

/* Config the Velocity closed loop gains in slot0 */
_falconA.config_kF(Constants.ShooterPIDIdx, Constants.ShooterKf, Constants.shooterTimeOut);
_falconA.config_kP(Constants.ShooterPIDIdx, Constants.ShooterKp, Constants.shooterTimeOut);
_falconA.config_kI(Constants.ShooterPIDIdx, Constants.ShooterKi, Constants.shooterTimeOut);
_falconA.config_kD(Constants.ShooterPIDIdx, Constants.ShooterKd, Constants.shooterTimeOut);


}



  public double getRPM() {
    //A method that returns the speed of the shooter motors
    return _falconA.getSelectedSensorVelocity();
  }

  public void setRPM(double rpm) {
    //These should work once we have set PID values <- Unfortunately we never got this working. We resorted to a complex series of commands 
    // to accomplish what a PID should have done (i.e. maintain motor RPM even during shooting.)
    //_falconA.set(ControlMode.Velocity, -rpm);
    //_falconB.set(ControlMode.Velocity, rpm);
    
    _falconA.set(ControlMode.PercentOutput, rpm);
    //_falconB.set(ControlMode.PercentOutput, rpm); <- _falconB is set to follow _falconA
    
  }

public void setRPM_PID(double rpm){
  _falconA.set(ControlMode.Velocity, rpm);
}

  //Aiming Functions <- never implemented. 
public double getDistance(){

  // Returns distance from goal in inches (in theory...)
  // Based on the target being at a known height and the camera at a known angle.
  // The most significant block to this working was getting a reliable signal from the limelight
  // often couldn't detect the target. Resulting in spotty reports to the NetworkTable
  NetworkTableEntry ty = table.getEntry("ty");
  double targetOffsetAngle_Vertical = ty.getDouble(0.0);
  if (targetOffsetAngle_Vertical == 0) { 
    return 0.0;
  } else {
  double angleToGoalDegrees = Constants.limelightMountAngle + targetOffsetAngle_Vertical;
  double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
  return (Constants.GoalHeight - Constants.limelightHeight)/Math.tan(angleToGoalRadians);
  }
}

public double getShooterSpeed(double distance) {
  //This function takes the distance and converts it via a function into shooter wheel speeds.
  //Currently assuming a linear relationship
    return (Constants.ShooterSpeedSlope * distance + Constants.ShooterSpeedIntercept);

}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // this is a good place to place code reporting the status of the subsystem.
    SmartDashboard.putNumber("Shooter A Velocity", _falconA.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter B Velocity", _falconB.getSelectedSensorVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
