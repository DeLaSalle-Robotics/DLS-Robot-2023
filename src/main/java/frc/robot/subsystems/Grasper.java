// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grasper extends SubsystemBase {  
  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid grasperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
   1,
   2);
  //private final VictorSPX _VictorSPX = new VictorSPX(1);
  
  //Declaration of subsystem and its components
  public Grasper() {
    grasperSolenoid.set(Value.kForward); // Should close the claw on startup
    
}

//Subsystem Methods:
  public void disableCompressor() {
    //Will turn off Compressor 
    phCompressor.disable();
  }

  public void enableCompressor() {
    //Will turn on compressor, if pressure is not maxed.
    phCompressor.enableDigital();
  }

  public void closeGrasp() {
    //Gets the climber motor position.
    grasperSolenoid.set(Value.kForward);
  }

  public void openGrasp() {
    //Allows climber position to be reset.
    grasperSolenoid.set(Value.kReverse);
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
