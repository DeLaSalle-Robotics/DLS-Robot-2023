// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.commands.TestCommand;

// Phoenix
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.node.DoubleNode;
// SparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// Wipilibj
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.Constants;


public class MiniArm extends SubsystemBase {
  private final WPI_TalonFX _armFalcon = new WPI_TalonFX(Constants.miniFalcon);
  //A through bore encorder that reports the arm position.
  private final DutyCycleEncoder _armEncoder = new DutyCycleEncoder(Constants.intakeArmEncoderChannel);
  Encoder m_encoder = new Encoder(7, 8,false, Encoder.EncodingType.k4X);
  PowerDistribution m_miniPD = new PowerDistribution(0, ModuleType.kCTRE); 
  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_wrist;
 
  private final DCMotor m_armGearbox = DCMotor.getFalcon500(1);
  private final double  m_armReduction = 20;
  private final double  m_armMass = 0.2;
  private final double m_armLength = Units.inchesToMeters(12);

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  private double priorArmVelocity = 0.0;

  // The P gain for the PID controller that drives this arm.
private static double kArmKp = 50.0;

private static double armPositionDeg = 75.0;
//Arm FeedForward
//ArmFeedforward feedforward = new ArmFeedforward(Constants.arm_kS, Constants.arm_kG, Constants.arm_kV);

//Simulation Code
 private final SingleJointedArmSim m_armSim =
 new SingleJointedArmSim(
   m_armGearbox, 
   m_armReduction,
  SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
   m_armLength,
    Units.degreesToRadians(-75), 
    Units.degreesToRadians(255),
     true,
     VecBuilder.fill(2 * Math.PI /2048.0)
     );
  private final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

     //Mechanism 2d Testing
     private final Mechanism2d m_mech2d = new Mechanism2d(60,60);
     private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30,30);
     private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
     private final MechanismLigament2d m_arm =
     m_armPivot.append(
      new MechanismLigament2d(
        "Arm",
        30,
        Units.radiansToDegrees(m_armSim.getAngleRads()),
        6,
        new Color8Bit(Color.kPurple)
      ));
     
     

  //Declaring the Subsystem \/
 public MiniArm() {
  _armFalcon.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalcon.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  m_encoder.setDistancePerPulse(2 * Math.PI /2048.0);
  
  if (!Preferences.containsKey(kArmPositionKey)) {
    Preferences.setDouble(kArmPositionKey, armPositionDeg);
  }
  if (!Preferences.containsKey(kArmPKey)) {
    Preferences.setDouble(kArmPKey, kArmKp);
  }

 // Put Mechanism 2d to SmartDashboard
 SmartDashboard.putData("Arm Sim", m_mech2d);
 m_armTower.setColor(new Color8Bit(Color.kBlue));

}
  

  public void ResetArmEncoder(){
    // Resets the encoder distance to 0 - Useful for fixing things n' stuff
    m_encoder.reset();
  }

  public void ArmMove(Double speed) {
    //This method sets the speed of the active intake mechanism
    _armFalcon.set(ControlMode.PercentOutput, speed * 0.5);
    this.ArmAngle();
  }

public void goToAngle(double armPositionDeg) {
  
    if (kArmKp != Preferences.getDouble(kArmPKey, kArmKp)) {
      kArmKp = Preferences.getDouble(kArmPKey, kArmKp);
      m_controller.setP(kArmKp);
    }
    var pidOutput = 
    m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(armPositionDeg));
    _armFalcon.setVoltage(pidOutput);
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

  public void ArmMoveVolts(double volt){
    var m_feedForward = this.getFeedForward(this.ArmAngle(), this.ArmLength());
    _armFalcon.setVoltage(volt);
    SmartDashboard.putNumber("Arm FeedForward", m_feedForward);
    SmartDashboard.putNumber("Arm Volts", volt);
  }

  public double ArmAngle() {
    //This method returns the arm angle in degrees
    double armAngle = m_encoder.getDistance();
    SmartDashboard.putNumber("Arm Angle", armAngle);
    return armAngle;
  }
  public double ArmVelocity() {
    //This method returns the arm angle in degrees
    double armRate = m_encoder.getRate();
    SmartDashboard.putNumber("Arm Rate", armRate);
    return(armRate);
  }
  public double ArmLength(){
    double armLength = 1;
    return armLength;
  }
  
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double armCurrent =  m_miniPD.getCurrent(2);
    //SmartDashboard.putNumber("Arm Current", armCurrent);
    //double armVoltage = m_miniPD.getVoltage();
    //SmartDashboard.putNumber("Arm Voltage", armVoltage);
    
    // Read Preferences for Arm setpoint and kP on entering Teleop
    
  }
  boolean testingKs;
  double startAngle;
  double currentVoltage = 0;
  double incrementAmount = 0.002;
  
  public void incrementVolts() {
    currentVoltage += incrementAmount;
    ArmAngle();
    this.ArmMoveVolts(currentVoltage);
  }
  public void stopVolts() {
    currentVoltage = 0;
    ArmAngle();
    this.ArmMoveVolts(currentVoltage);
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(_armFalcon.get() * RobotController.getBatteryVoltage());

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

  


}
