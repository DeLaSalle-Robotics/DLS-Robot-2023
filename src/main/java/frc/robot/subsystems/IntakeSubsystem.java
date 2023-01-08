// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSPX _snowblower = new VictorSPX(Constants.intake_snowblower); //angle
  private final VictorSPX _775 = new VictorSPX(Constants.intake_775); //Intake wheels
  private final CANSparkMax _neo = new CANSparkMax(Constants.intake_neo, MotorType.kBrushless); //Indexer/preindexer

  private final DutyCycleEncoder _intakeArmEncoder = new DutyCycleEncoder(Constants.intakeArmEncoderChannel);

 //Declaring the Subsystem \/
 public IntakeSubsystem() {
   //Putting the PID constants on the SmartDashboard is a good way to tune them.
   //Although it is not ideal to leave them there for the competion.
  SmartDashboard.putNumber("Controller P", Constants.IntakePID_kP);
  SmartDashboard.putNumber("Controller I", Constants.IntakePID_kI);
  SmartDashboard.putNumber("Controller D", Constants.IntakePID_kD);
}



  public void spinIntake(Double speed) {
    //This method sets the speed of the active intake mechanism
    _775.set(ControlMode.PercentOutput, speed);
  }

  public void moveIntakeAngle(Double speed) {
    //This method moves the angle of the active intake mechanism
    _snowblower.set(ControlMode.PercentOutput, speed);
  }

  public void spinIndexer(Double speed) {
    //This method sets the speed of the indexer motor
    _neo.set(speed);
  }

  public double getArmEncoder() {
    //This method returns the position of the encoder
    return(_intakeArmEncoder.getAbsolutePosition());
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
