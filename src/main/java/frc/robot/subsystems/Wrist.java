// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends PIDSubsystem {  
  private final VictorSPX __wristControl = new VictorSPX(Constants.wrist775ID); //Intake wheels
  private final CANSparkMax _snowblower = new CANSparkMax(Constants.climber_snowblower, MotorType.kBrushed);

  private final SimpleMotorFeedforward m_wristFeedforward =
      new SimpleMotorFeedforward(Constants.WristkSVolts, Constants.kVVoltSecondsPerRotation);
  //private final VictorSPX _VictorSPX = new VictorSPX(1);
  
  //Declaration of subsystem and its components
  public Wrist(double target) {

    super(new PIDController(Constants.WristConstants_kP, 
    Constants.WristConstants_kI, 
    Constants.WristConstants_kD));

    getController().setTolerance(Constants.kWristToleranceRPS);
// initalize whatever the gyro? Probably best in robotcontainer.
    m_wristGyro.setDistancePerPulse(Constants.kEncoderDistancePerPulse); // Whatever we use to measure the wrist angle

    setSetpoint(target);

    

  }


  @Override
/* In the PIDSubSystem class there is a method called useOutput. We need to 
 override that method to replace it with the details of our mechanism. That
is what the @Override does.
*/
  public void useOutput(double output, double setpoint) {

    __wristControl.setVoltage(output + m_wristFeedforward.calculate(setpoint));

  }


  @Override
/*
 * Similar to the useOutput method, we need to specify the means of controlling the wrist.
 * 
 */
  public double getMeasurement() {

    return m_wristEncoder.getAngle();

  }


  public boolean atSetpoint() {

    return m_controller.atSetpoint(); //Determined by the tolerances defined within the subsystem

  }

  //These constants were initially for tuning a PID controller. This was not properly implemented
    
}

//Subsystem Methods:
  public void controlWrist(double speed) {
    //Sets the climber motor speed
    __wristControl.set(VictorSPXControlMode.PercentOutput ,speed);
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
