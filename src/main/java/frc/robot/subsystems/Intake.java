// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {  
  private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid grasperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
   1,
   2);
   private final DoubleSolenoid twisterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
   3,
   4);
  private final CANSparkMax _IntakeNeo550 = new CANSparkMax(Constants.IntakeID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final PowerDistribution pdh; 
  private boolean IntakeVertical;
  //Declaration of subsystem and its components
  public Intake() {
    grasperSolenoid.set(Value.kForward); // Should close the claw on startup
    twisterSolenoid.set(Value.kForward); // Should put claw horizontal to node
    _IntakeNeo550.setSmartCurrentLimit(10);
    _IntakeNeo550.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putBoolean("Claw Vertical", false);
    pdh = new PowerDistribution(1, ModuleType.kRev);
    SmartDashboard.putBoolean("Have Piece", true);
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
    SmartDashboard.putBoolean("Have Piece", true);
  }

  public void openGrasp() {
    //Allows climber position to be reset.
    grasperSolenoid.set(Value.kReverse);
    SmartDashboard.putBoolean("Have Piece", false);
  }

  public void intakeHorizontal(){
    //Turn the claw -may be punmatics or maybe a motor
    twisterSolenoid.set(Value.kForward);
    SmartDashboard.putBoolean("Intake Vertical", false);
  }
  public void intakeVertical(){
    //Turn the claw -may be punmatics or maybe a motor
    twisterSolenoid.set(Value.kReverse);
    SmartDashboard.putBoolean("Intake Vertical", true);
  }

  public double getIntakeCurrent() {
    return pdh.getCurrent(Constants.intakeChannel);
    
  }

public void spinIntake(double speed){
  //Spin the wheels at set speed
  //Need to monitor current going to intake motor, once a game piece is acquired, we should stop spinning and close clamp
  //This may be done at the Command level.
  _IntakeNeo550.set(speed);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double intakeCurrent = pdh.getCurrent(Constants.intakeChannel);
    SmartDashboard.putNumber("Intake Current", intakeCurrent);
    if (intakeCurrent > Constants.intakeCurrentThreshold){
      this.spinIntake(0.0);
      SmartDashboard.putBoolean("Have Piece", true);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
