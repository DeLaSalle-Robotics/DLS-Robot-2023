// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Phoenix
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// SparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// Wipilibj
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
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


public class Arm extends SubsystemBase {
  private final WPI_TalonFX _armFalconR = new WPI_TalonFX(Constants.armFalconRightID);
  private final WPI_TalonFX _armFalconL = new WPI_TalonFX(Constants.armFalconLeftID);
  private final WPI_TalonFX _armExtend = new WPI_TalonFX(Constants.armExtendID);
  private final Encoder m_encoder = new Encoder(7, 8,false, Encoder.EncodingType.k4X); //<-- put channels in the Constants class

  //Setting Initial State
  private final DCMotor m_armGearbox = DCMotor.getFalcon500(2);
  private final double  m_armReduction = 280; // <-- Should be in the Constants class
  private final double  m_armMass = 0.2; //<-- Needs to be updated for acutal arm
  private final double m_armLength = Units.inchesToMeters(24); //<-- Will be provied by a method
  private double priorArmVelocity = 0.0;
  private static double armPositionDeg = 75.0; //<-- Whatever position keeps us within frame parameter

  // The P gain for the PID controller that drives this arm. May need a full PID <-- although this is likely in a command
  private static double kArmKp = 50.0;
  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  //Simulation Code
 private final SingleJointedArmSim m_armSim =
 new SingleJointedArmSim(
   m_armGearbox, 
   m_armReduction,
  SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
   m_armLength, // <-- This might present a problem trying to alter the arm length in simulation
    Units.degreesToRadians(-75),  //<-- These need to be defined on the robot
    Units.degreesToRadians(255), //<-- These need to be defined on the robot
     true,
     VecBuilder.fill(2 * Math.PI /2048.0)
     );
  private final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

       //Mechanism 2d Testing
       private final Mechanism2d m_mech2d = new Mechanism2d(60,60);
       private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30,30); //<-- Probably be wise to get the height here reasonably close to reality
       private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));//<-- Probably be wise to get the height here reasonably close to reality
       private final MechanismLigament2d m_arm =
       m_armPivot.append(
        new MechanismLigament2d(
          "Arm",
          30, //<-- needs to be variable
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kPurple)
        ));
  

 //Declaring the Subsystem \/
 public Arm() {
  _armFalconR.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconR.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  _armFalconR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
  _armFalconL.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalconL.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  _armFalconL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 30));
  _armFalconR.follow(_armFalconL);
  _armFalconR.setInverted(true); // Not sure if this is right
  
  _armExtend.configFactoryDefault();
  _armExtend.setNeutralMode(NeutralMode.Brake);
  _armExtend.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);

  m_encoder.setDistancePerPulse(2 * Math.PI /2048.0);

   // Put Mechanism 2d to SmartDashboard
 SmartDashboard.putData("Arm Sim", m_mech2d);
 m_armTower.setColor(new Color8Bit(Color.kBlue));

if (!Preferences.containsKey(kArmPositionKey)) {
    Preferences.setDouble(kArmPositionKey, armPositionDeg);
  }
if (!Preferences.containsKey(kArmPKey)) {
    Preferences.setDouble(kArmPKey, kArmKp);
  }

}


  public void ResetArmEncoder(){
    // Resets the encoder distance to 0 - Useful for fixing things n' stuff
    //_armEncoder.reset();
  }

  public void ArmMove(Double speed) {
    //This method sets the speed of the active intake mechanism
    _armFalconL.set(ControlMode.PercentOutput, speed);
  }

  public void ArmMoveVolts(double volt){
    double m_feedForward = this.getFeedForward(this.ArmAngle(), this.ArmLength());
    _armFalconL.setVoltage(volt + m_feedForward);
    SmartDashboard.putNumber("Arm FeedForward", m_feedForward);
    SmartDashboard.putNumber("Arm Volts", volt);
  }

  public double ArmAngle() {
    //This method returns the arm angle in degrees
    double vertical_radian = m_encoder.getDistance(); //<-- Need to confirm the units
    return(Math.toDegrees(vertical_radian));
  }
  public double ArmVelocity() {
    //This method returns the arm angle in degrees
    double armRate = m_encoder.getRate();
    return(armRate);
  }


  public double ArmLength(){
    double armLength_clicks = _armExtend.getSelectedSensorPosition();
    //Conversion figure to convert length to sensor position
    double armlength_m = 0 * armLength_clicks + 42; //<-- needs to be determined.
    return armlength_m;
  }

  public void ArmExtend(Double speed) {
    //This method sets the speed of the arm extension motor
    _armExtend.set(speed);
  }

private double ArmComCalc(double armLength){
  double Arm_Com = armLength; //Function to convert armlength to Center of Mass distance from pivot
  return Arm_Com;
}

public double getFeedForward(double armAngle, double armLength){
  double Arm_Com = ArmComCalc(this.ArmLength());  //Get the Center of Mass (Com)
  double feedForward = Constants.arm_Kg * Math.cos(armAngle) * Arm_Com+ //Static Torque Component
                      Constants.arm_Ks +                                //Static Motor Component
                      Constants.arm_Kv * this.ArmVelocity() +           //Torque of friction
                      Constants.arm_Ka * Arm_Com* Arm_Com * (this.ArmVelocity() - priorArmVelocity)/0.02; //Angular Momentum Calculation
  priorArmVelocity = this.ArmVelocity();
  //double feedForward = 0.0;
return(feedForward);
}

public void findArmLocation(){
  //This method needs to find a way to set the arms position. Could move slowly until it hits the edge
  // while monitorting current, then set the encoder once a threshold is hit.

  // Move slowly and monitor current
  

  // Wait for the threshold
  
  // Set the encoder

}

public void setArmLength(double armLength) {
  //This method moves arm to set length - Probably better as a command.
}

public void armLengthFineControl() {
  //This method/command should provide fine control of arm extension to allow corrective aiming of initial position
}

public void targetingPose() {
  //This needs to be a command that can take targeting information and set arm length and chassis position to faciliate scoring at selected target
  //Also need a method of target selection, thinking about a grid of booleans on the SmartDashboard
}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(_armFalconL.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  //These methods are used to characterize the arm constants
  boolean testingKs;
  double startAngle;
  double currentVoltage = 0;
  double incrementAmount = 0.002;
  
  public void incrementVolts() {
    currentVoltage += incrementAmount;
    SmartDashboard.putNumber("ArmAngle", ArmAngle());
    SmartDashboard.putNumber("ArmRate", ArmVelocity());
    SmartDashboard.putNumber("Volts", currentVoltage);
    
    _armFalconL.setVoltage(currentVoltage);
  }
  public void armSetVolts(double volts){
    _armFalconL.setVoltage(volts);
  }

  public void stopVolts() {
    _armFalconL.setVoltage(0);
  }
}
