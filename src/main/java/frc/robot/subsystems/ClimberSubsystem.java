// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {  
  private final WPI_TalonFX _falconA = new WPI_TalonFX(Constants.climber_falcon);
  private final CANSparkMax _snowblower = new CANSparkMax(Constants.climber_snowblower, MotorType.kBrushed);

  //private final VictorSPX _VictorSPX = new VictorSPX(1);
  
  //Declaration of subsystem and its components
  public ClimberSubsystem() {
    _falconA.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
    _falconA.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our climber. Other option is coast.
    _falconA.setSelectedSensorPosition(0.0); //Setting the encoder position to 0

    //These constants were initially for tuning a PID controller. This was not properly implemented
    SmartDashboard.putNumber("Winch Controller P", Constants.IntakePID_kP);
    SmartDashboard.putNumber("Winch Controller I", Constants.IntakePID_kI);
    SmartDashboard.putNumber("Winch Controller D", Constants.IntakePID_kD);
    
}

//Subsystem Methods:
  public void spinWinch(double speed) {
    //Sets the climber motor speed
    _falconA.set(speed);
  }

  public void moveClimber(double speed) {
    //Sets the speed of the motor that pulls the climbing arms back
    _snowblower.set(speed);
  }

  public double getWinchEncoder() {
    //Gets the climber motor position.
    return _falconA.getSelectedSensorPosition();
  }

  public void resetWinchEncoder() {
    //Allows climber position to be reset.
    _falconA.setSelectedSensorPosition(0.0);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run and will post the climber position to the SmartDashBoard
    //SmartDashboard.putNumber("Climber Encoder", _snowblower.getEncoder(Type.kQuadrature, 8192).getPosition());
    SmartDashboard.putNumber("Climber Falcon Encoder", getWinchEncoder());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
