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
  private final WPI_TalonFX _upFalcon = new WPI_TalonFX(Constants.upFalconID);
  private final WPI_TalonFX _outFalcon = new WPI_TalonFX(Constants.outFalconID);
 
  // Encoder for arm position/orientation
  private final DutyCycleEncoder _armPosEncoder = new DutyCycleEncoder(Constants.intakeArmEncoderChannel);

 //Declaring the Subsystem \/
 public Arm() {
  _upFalcon.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _upFalcon.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our climber. Other option is coast.
  _upFalcon.setSelectedSensorPosition(0.0); //Setting the encoder position to 0: may want to tie this to a limit switch?
  _outFalcon.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _outFalcon.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our climber. Other option is coast.
  _outFalcon.setSelectedSensorPosition(0.0); //Setting the encoder position to 0: may want to tie this to a limit switch?


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
    _upFalcon.set(ControlMode.PercentOutput, speed);
    _outFalcon.set(ControlMode.PercentOutput, (speed * -1.0)); // Inverse of upFalcon
  }

  public double ArmAngle() {
    // This method returns the arm angle in degrees
    double armPos = _armPosEncoder.get(); // Returns in rotations - multiply by 360 to convert to degrees
    double armDegrees = armPos * 360;
    return armDegrees;
  }

  public void ArmExtend(Double speed) {
    //This method sets the speed of the arm extension motor
    _outFalcon.set(speed);
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
