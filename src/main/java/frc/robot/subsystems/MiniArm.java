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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Wipilibj
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constants
import frc.robot.Constants;


public class MiniArm extends SubsystemBase {
  private final WPI_TalonFX _armFalcon = new WPI_TalonFX(Constants.miniFalcon);
  //A through bore encorder that reports the arm position.
  private final DutyCycleEncoder _armEncoder = new DutyCycleEncoder(Constants.intakeArmEncoderChannel);
  Encoder encoder = new Encoder(7, 8,false, Encoder.EncodingType.k4X);
  PowerDistribution m_miniPD = new PowerDistribution(0, ModuleType.kCTRE); 
  //Declaring the Subsystem \/
 public MiniArm() {
  _armFalcon.configFactoryDefault(); //Resets any preexisting settings - good practice to prevent things from breaking unexpectedly.
  _armFalcon.setNeutralMode(NeutralMode.Brake); //Setting neutral mode to break, which is good for our arm. Other option is coast.
  encoder.setDistancePerPulse(2 * Math.PI /2048.0);
  
   //Putting the PID constants on the SmartDashboard is a good way to tune them.
   //Although it is not ideal to leave them there for the competion.
  SmartDashboard.putNumber("Controller P", Constants.IntakePID_kP);
  SmartDashboard.putNumber("Controller I", Constants.IntakePID_kI);
  SmartDashboard.putNumber("Controller D", Constants.IntakePID_kD);
}


  public void ResetArmEncoder(){
    // Resets the encoder distance to 0 - Useful for fixing things n' stuff
    _armEncoder.reset();
  }

  public void ArmMove(Double speed) {
    //This method sets the speed of the active intake mechanism
    _armFalcon.set(ControlMode.PercentOutput, speed);
    
  }

  public void ArmMoveVolts(Double volt){
    _armFalcon.setVoltage(volt);
  }

  public double ArmAngle() {
    //This method returns the arm angle in degrees
    double armAngle = encoder.getDistance();
    SmartDashboard.putNumber("Arm Angle", armAngle);
    return armAngle;
  }
  public double ArmVelocity() {
    //This method returns the arm angle in degrees
    double armRate = encoder.getRate();
    SmartDashboard.putNumber("Arm Rate", armRate);
    return(armRate);
  }

  

  public double getArmEncoder() {
    //This method returns the position of the encoder
    return(_armEncoder.getAbsolutePosition());
  }

  public double[] predictVelAndAccel(double _time, double[] velArray, double[] accelArray){
    // Takes in time and returns what the velocity and acceleration should be
    double[] output = new double[2];
    int index = (int)Math.round(_time * 50);
    output[0] = velArray[index];
    output[1] = accelArray[index];
    return output;
  }

  public double[][] getPredictions(double endAngle, double endLength) {
    double startAngle = ArmAngle();
    double startLength = getArmEncoder(); // Check this

    // Get constant length to trapezoid
    double constTimeAccel = Constants.maxVelocityMetersPerSecond / Constants.maxAccelerationMetersPerSecondSq;
    double constLenToTrapezoid = constTimeAccel * Constants.maxVelocityMetersPerSecond;

    double angleChange = endAngle - startAngle;
    double absAngleChange = Math.abs(angleChange);

    // Triangles
    if (absAngleChange < constLenToTrapezoid) {

      // Get total time
      double timeAccel = Math.sqrt(absAngleChange / Constants.maxAccelerationMetersPerSecondSq);
      double time = timeAccel * 2;

      // Defines the arrays that will be returned
      double[] velArray = new double[(int)Math.ceil(time * 50)];
      double[] accelArray = new double[velArray.length];

      // Set values in the arrays
      for(int i = 0;i < velArray.length;i++) {
        double timeSeconds = i/50;
        if(timeSeconds <= timeAccel) { // Before peak
          accelArray[i] = Constants.maxAccelerationMetersPerSecondSq;
          velArray[i] = timeSeconds * Constants.maxAccelerationMetersPerSecondSq;
        } else { // After peak
          accelArray[i] = -Constants.maxAccelerationMetersPerSecondSq;
          velArray[i] = (time - timeSeconds) * Constants.maxAccelerationMetersPerSecondSq;
        }
      }

      // Return the arrays
      return new double[][] {velArray, accelArray};
    } else { // Trapezoid

      double distanceMinusConstant = absAngleChange - constLenToTrapezoid;
      double timeCoasting = distanceMinusConstant / Constants.maxVelocityMetersPerSecond;
      double totalTime = timeCoasting + constTimeAccel * 2;

      double[] velArray = new double[(int)Math.ceil(totalTime * 50)];
      double[] accelArray = new double[velArray.length];

      // Set values in arrays
      for (int i = 0; i < velArray.length; i++){
        double timeSeconds = i / 50;

        if(timeSeconds <= constTimeAccel) { // Before peak
          accelArray[i] = Constants.maxAccelerationMetersPerSecondSq;
          velArray[i] = timeSeconds * Constants.maxAccelerationMetersPerSecondSq;
        } else if (timeSeconds >= (totalTime - constTimeAccel)){ // After peak
          accelArray[i] = -Constants.maxAccelerationMetersPerSecondSq;
          velArray[i] = (totalTime - timeSeconds) * Constants.maxAccelerationMetersPerSecondSq;
        } else { // At peak
          accelArray[i] = 0;
          velArray[i] = Constants.maxVelocityMetersPerSecond;
        }
      }

      // Return the arrays
      return new double[][] {velArray, accelArray};
    }
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double armCurrent =  m_miniPD.getCurrent(2);
    SmartDashboard.putNumber("Arm Current", armCurrent);
    double armVoltage = m_miniPD.getVoltage();
    SmartDashboard.putNumber("Arm Voltage", armVoltage);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
