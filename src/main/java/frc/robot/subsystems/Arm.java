// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Unused imports
//import com.revrobotics.AbsoluteEncoder;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DutyCycle;
//import edu.wpi.first.wpilibj.Preferences;

// Phoenix
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// Wipilibj
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.Constants;
import frc.robot.Robot;


public class Arm extends SubsystemBase {
  private final WPI_TalonFX _armFalconR = new WPI_TalonFX(Constants.armFalconRightID);
  private final WPI_TalonFX _armFalconL = new WPI_TalonFX(Constants.armFalconLeftID);
  
  private final Encoder m_encoder = new Encoder(7, 8,false, Encoder.EncodingType.k4X); //<-- put channels in the Constants class
  private final DutyCycleEncoder m_abEncoder = new DutyCycleEncoder(2);
  //Setting Initial State for Arm Simulation
  private final DCMotor m_armGearbox = DCMotor.getFalcon500(2);
  private final double  m_armReduction = 280; // <-- Should be in the Constants class
  private final double  m_armMass = 7; //<-- Needs to be updated for acutal arm - guesstimate in kg
  private final double m_armLength = Units.inchesToMeters(24); //<-- Will be provied by a method
  private double priorArmVelocity = 0.0;
  private static double armPositionRad = Math.PI/4; //<-- Whatever position keeps us within frame parameter

  // The P gain for the PID controller that drives this arm. May need a full PID <-- although this is likely in a command
  private static double kArmKp = 50.0;
  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  private DigitalInput magSwitch = new DigitalInput(0);

  //Simulation Code
 private final SingleJointedArmSim m_armSim =
 new SingleJointedArmSim(
   m_armGearbox, 
   m_armReduction,
  SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
   m_armLength, // <-- This might present a problem trying to alter the arm length in simulation
    Units.degreesToRadians(-75),  //<-- These need to be defined on the robot
    Units.degreesToRadians(255), //<-- These need to be defined on the robot
     //m_armMass,
     true,
     VecBuilder.fill(2 * Math.PI /2048.0)
     );

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

       //Mechanism 2d Sim Setup
       private final Mechanism2d m_mech2d = new Mechanism2d(3,3);
       private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 1.5,0.35); 
       private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", .35, -90));
       private final MechanismLigament2d m_arm =
       m_armPivot.append(
        new MechanismLigament2d(
          "Arm",
          1.2, //<-- needs to be variable
          m_armSim.getAngleRads(),
          6,
          new Color8Bit(Color.kPurple)
        ));
  

 //Constructing the Subsystem \/
 public Arm() {
  _armFalconR.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconR.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  _armFalconL.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconL.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  if (Constants.limitFalcons) {
    _armFalconR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
    _armFalconL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
  }
  _armFalconR.follow(_armFalconL);
  _armFalconR.setInverted(true); 

  _armFalconL.configOpenloopRamp(Constants.Arm_Ramp);
  _armFalconR.configOpenloopRamp(Constants.Arm_Ramp);

   m_encoder.setDistancePerPulse(2 * Math.PI /2048.0);

   // Put Mechanism 2d to SmartDashboard
 SmartDashboard.putData("Arm Sim", m_mech2d);
 m_armTower.setColor(new Color8Bit(Color.kBlue));

}

public double GetABencoder(){
  if (Constants.verbose){
    SmartDashboard.putNumber("Absolute", m_abEncoder.getAbsolutePosition());
  }
  return m_abEncoder.getAbsolutePosition();
}


  public void ResetArmEncoder(){
    m_encoder.reset();
  }

  public void ArmMove(Double speed) {
    //This method sets the speed of the active intake mechanism
   /* 
    if (Math.toDegrees(this.ArmAngle()) < -35 & speed < 0) {
      speed = 0.0;
    }
  
  if (Math.toDegrees(this.ArmAngle())> 200 & speed > 0) {
    speed = 0.0;
  }
  */
    _armFalconL.set( speed);
  }

  public void ArmMoveVolts(double volt){
    var m_feedForward = this.getFeedForward(this.ArmAngle());
    _armFalconL.setVoltage(volt + m_feedForward);
   
  }

  public double ArmAngle() {
    //This method returns the arm angle in radians
    if (Robot.isReal()) {
      double vertical_radian = m_encoder.getDistance(); //<-- Returns in radians
      double armAngleCorrection = Math.toRadians(70);
      double correctedAngle = vertical_radian - armAngleCorrection;
      return(Math.toDegrees(this.GetABencoder())); 
    } else {
      return m_armSim.getAngleRads();
    }
  }
  public double ArmVelocity() {

    //This method returns the arm angle in degrees
    double armRate = m_encoder.getRate();
    return(armRate);
  }

public double getFeedForward(double armAngle){
  double Arm_Com = SmartDashboard.getNumber("CoM", 0.3);  //Get the Center of Mass (CoM)
  double curVel = this.ArmVelocity();
  double curDir;
  double Ks;
  if (curVel > 0){ curDir = 1;}
    else{ curDir = -1;}
  if (Robot.isReal()) {Ks = Constants.arm_Ks;} else {Ks = 0;}
  double feedForward = Constants.arm_Kg * Math.cos(armAngle) * Arm_Com+ //Static Torque Component
                      Ks * curDir +                                //Static Motor Component <- This needs to flip depending on the direction
                      Constants.arm_Kv * curVel +           //Torque of friction
                      Constants.arm_Ka * Arm_Com* Arm_Com * (this.ArmVelocity() - priorArmVelocity)/0.02; //Angular Momentum Calculation
  priorArmVelocity = this.ArmVelocity();
  //double feedForward = 0.0;
  if (Constants.verbose) {
  SmartDashboard.putNumber("Ks", Ks * curDir);
  SmartDashboard.putNumber("Kg", Constants.arm_Kg * Math.cos(armAngle) * Arm_Com);
  SmartDashboard.putNumber("Kv", Constants.arm_Kv * curVel);
  SmartDashboard.putNumber("Ka", Constants.arm_Ka * Arm_Com* Arm_Com * (this.ArmVelocity() - priorArmVelocity)/0.02);
  SmartDashboard.putNumber("FeedForward", feedForward);
  }
return(feedForward);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Check if the magnetic sensor passes the arm- 
    if (magSwitch.get()) {
      //m_encoder.reset();
    }
    
    SmartDashboard.putNumber("Current Arm Angle", Math.toDegrees(this.ArmAngle()));
    SmartDashboard.putNumber("Absolute Arm Angle", Math.toDegrees(this.GetABencoder()));
    //Create check on arm position and limit over extension <- create warning for Smart dashboard.
    //Pretty complicated because it is dependent on current length
   
  }
  @Override
  public void simulationPeriodic() {
    
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(_armFalconL.get() * RobotController.getBatteryVoltage());
    if (Constants.verbose) {SmartDashboard.putNumber("Sim Input", _armFalconL.get());}
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_encoderSim.setRate(m_armSim.getVelocityRadPerSec());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_arm.setLength(SmartDashboard.getNumber("Arm Length", 0.3));
  }

  //These methods are used to characterize the arm constants
  boolean testingKs;
  double startAngle;
  double currentVoltage = 0;
  double incrementAmount = 0.002;
  
  public void incrementVolts() {
    currentVoltage += incrementAmount;
    if (Constants.verbose) {
    SmartDashboard.putNumber("ArmAngle", ArmAngle());
    SmartDashboard.putNumber("ArmRate", ArmVelocity());
    SmartDashboard.putNumber("Volts", currentVoltage);
    }
    _armFalconL.setVoltage(currentVoltage);
  }
  public void armSetVolts(double volts){
    _armFalconL.setVoltage(volts);
  }

  public void stopVolts() {
    _armFalconL.setVoltage(0);
  }
}
