// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Phoenix
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// SparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Wipilibj
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  private final WPI_TalonFX _armFalconR = new WPI_TalonFX(Constants.armFalconRightID);
  private final WPI_TalonFX _armFalconL = new WPI_TalonFX(Constants.armFalconLeftID);
  private final WPI_TalonFX _armExtend = new WPI_TalonFX(Constants.armExtendID);
  //A through bore encorder that reports the arm position.
  private final DutyCycleEncoder _armEncoder = new DutyCycleEncoder(Constants.intakeArmEncoderChannel);

 //Declaring the Subsystem \/
 public Arm() {
  _armFalconR.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconR.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  _armFalconL.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconL.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  _armFalconR.follow(_armFalconL);
  _armFalconR.setInverted(true); // Not sure if this is right
  _armExtend.configFactoryDefault();
  _armExtend.setNeutralMode(NeutralMode.Brake);

   //Putting the PID constants on the SmartDashboard is a good way to tune them.
   //Although it is not ideal to leave them there for the competion.
  SmartDashboard.putNumber("Controller P", Constants.IntakePID_kP);
  SmartDashboard.putNumber("Controller I", Constants.IntakePID_kI);
  SmartDashboard.putNumber("Controller D", Constants.IntakePID_kD);
}


  public void ResetArmEncoder(){
    // Resets the encoder distance to 0 - Useful for fixing things n' stuff
    _armPosEncoder.reset();
  }

  public void ArmMove(Double speed) {
    //This method sets the speed of the active intake mechanism
    _armFalconL.set(ControlMode.PercentOutput, speed);
  }

  public void ArmMoveVolts(Double volt){
    _armFalconL.setVoltage(volt);
  }


  public double ArmAngle() {
    //This method returns the arm angle in degrees
    double encoderReading = _armEncoder.getAbsolutePosition();
    double vertical_radian = Math.asin(vertical_dist/Constants.stage1Length);
    return(Math.toDegrees(vertical_radian));
  }

  public void ArmExtend(Double speed) {
    //This method sets the speed of the arm extension motor
    _armExtend.set(speed);
  }

  public double getArmEncoder() {
    //This method returns the position of the encoder
    return(_armPosEncoder.getAbsolutePosition());
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
