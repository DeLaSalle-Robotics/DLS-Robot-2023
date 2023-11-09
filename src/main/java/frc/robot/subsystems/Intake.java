// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Unused imports
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import com.revrobotics.SparkMaxRelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {  
  private final PneumaticHub m_ph = new PneumaticHub(9);
  private final DoubleSolenoid grasperSolenoid = m_ph.makeDoubleSolenoid(0, 3);//
  private final DoubleSolenoid twisterSolenoid = m_ph.makeDoubleSolenoid(1, 2);
  private final CANSparkMax _IntakeNeo550 = new CANSparkMax(Constants.IntakeID, CANSparkMax.MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  private final PowerDistribution pdh; 
  
  private boolean IntakeVertical;
  private boolean grasperOpen;
  

  //Declaration of subsystem and its components
  public Intake() {
    
    grasperSolenoid.set(Value.kForward); // Should close the claw on startup
    twisterSolenoid.set(Value.kForward); // Should put claw horizontal to node
    //_IntakeNeo550.setSmartCurrentLimit(10);
    _IntakeNeo550.setIdleMode(IdleMode.kBrake);
    m_encoder = _IntakeNeo550.getEncoder();
    SmartDashboard.putBoolean("Claw Vertical", false);
    SmartDashboard.putBoolean("Claw Open", grasperOpen);
    pdh = new PowerDistribution(1, ModuleType.kRev);
    SmartDashboard.putBoolean("Have Piece", true);
}

//Subsystem Methods:
  public void disableCompressor() {
    //Will turn off Compressor 
    //phCompressor.disable();
    m_ph.disableCompressor();
  }

  public void enableCompressor() {
    //Will turn on compressor, if pressure is not maxed.
    //phCompressor.enableDigital();
    m_ph.enableCompressorDigital();
  }

  public void closeGrasp() {
    //Close Grapser Position.
    twisterSolenoid.set(DoubleSolenoid.Value.kReverse);
    grasperOpen = false;
    if (Constants.verbose) {SmartDashboard.putBoolean("Claw Open", grasperOpen);}
  }

  public void openGrasp() {
    
    if (grasperSolenoid.get() == DoubleSolenoid.Value.kForward){
      grasperSolenoid.set(DoubleSolenoid.Value.kReverse);
      grasperOpen = false;
      
    } else {
      grasperSolenoid.set(DoubleSolenoid.Value.kForward);
      grasperOpen = true;
      
    }
    SmartDashboard.putBoolean("Claw Open", grasperOpen);
    if (Constants.verbose) {SmartDashboard.putBoolean("Claw Open", grasperOpen);}
  }
  public void intakeFlip(){
    //Turn the claw -may be punmatics or maybe a motor
    if (twisterSolenoid.get() == DoubleSolenoid.Value.kForward){
      twisterSolenoid.set(DoubleSolenoid.Value.kReverse);
      IntakeVertical = false;
      
    } else {
      twisterSolenoid.set(DoubleSolenoid.Value.kForward);
      IntakeVertical = true;
      
    }
    if (Constants.verbose) {SmartDashboard.putBoolean("Intake Vertical", IntakeVertical);}
  }


  

  public void intakeHorizontal(){
    //Turn the claw 
    twisterSolenoid.set(DoubleSolenoid.Value.kForward);
    IntakeVertical = false;
    if (Constants.verbose) {SmartDashboard.putBoolean("Intake Vertical", IntakeVertical);}
  }
  public void intakeVertical(){
    //Turn the claw
    twisterSolenoid.set(DoubleSolenoid.Value.kReverse);
    IntakeVertical = true;
    if (Constants.verbose) {SmartDashboard.putBoolean("Intake Vertical", IntakeVertical);}
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

public void stopIntake() {
  _IntakeNeo550.set(0);
}

public double spinVelocity() {
  return m_encoder.getVelocity();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (Constants.verbose)
    //   //{SmartDashboard.putNumber("Intake Voltage", _IntakeNeo550.getBusVoltage());
    //   SmartDashboard.putNumber("Compress Current", phCompressor.getCurrent());
    // }
    //   SmartDashboard.putNumber("Intake Speed", _IntakeNeo550.get());
    //   if (SmartDashboard.getBoolean("Compressor", true)){
    //     phCompressor.enableDigital();
    //   } else {
    //     phCompressor.disable();
    //   }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void scoreHigh(){
    SmartDashboard.putString("Score Type", "High");
  }
  public void scoreMid(){
    SmartDashboard.putString("Score Type", "Mid");
  }
  public void scoreLow(){
    SmartDashboard.putString("Score Type", "Low");
  }
  public void scoreCone(){
    SmartDashboard.putString("Piece Type", "Cone");
  }
  public void scoreCube(){
    SmartDashboard.putString("Piece Type", "Cube");
  }
}
