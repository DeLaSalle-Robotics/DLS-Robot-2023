package frc.robot.subsystems;

// Unused imports
//import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmExtend extends SubsystemBase{
    private final WPI_TalonFX _armExtend = new WPI_TalonFX(Constants.armExtendID);
    //Simuation Parts
  private final TalonFXSimCollection sim_armExtend = _armExtend.getSimCollection();
  private double kCountsPerRev = 2048;
private final ElevatorSim m_armSim = 
new ElevatorSim(
    DCMotor.getFalcon500(1),
    12.0,7.0,0.02, 0.3683,1.397,false,VecBuilder.fill(0.01)
);

  public ArmExtend(){
    _armExtend.configFactoryDefault();
    _armExtend.setNeutralMode(NeutralMode.Brake);
    _armExtend.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);
    if( Constants.limitFalcons) { 
      _armExtend.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
       30, 0, 30));
      }
    _armExtend.setSelectedSensorPosition(0);
  
  }
  public double getArmLength(){
    if (Robot.isReal()){
    double armLength_clicks = _armExtend.getSelectedSensorPosition();
    //Conversion figure to convert length to sensor position
    //Falcon 500 has 2048 "clicks" per full revolution
    double armlength_m = 7.305E-6 * armLength_clicks + .3; //<-- Estimate from radius needs to be confirmed.
    return armlength_m;} else {
       return m_armSim.getPositionMeters();
    }
  }

  public void ArmExtention(double speed) {
    //This method sets the speed of the arm extension motor
    _armExtend.set(speed);
  }
  
  public void ArmExtentionVolts(double volts) {
    //This method sets the speed of the arm extension motor
    _armExtend.setVoltage(volts);
  }

  public double ArmComCalc(){

    //Function to convert total arm length to Center of Mass distance from pivot <- Derived from measurments
      
    double Arm_Com = this.getArmLength() * 0.5711 - 0.0639; 
    return Arm_Com;
  }

public boolean moveToState(String key){
  //2048 clicks in a rotation
  double armLength_clicks = _armExtend.getSelectedSensorPosition();
  double armBuffer = 500;
  double armSpeed = 0.5;
  double requestedArmPosition = 0;
  switch(key){
    case "low":
    requestedArmPosition = 0;  

    case "mid":
    requestedArmPosition = 17000;
   
    case "high":
    requestedArmPosition = 28000;
  }


if (requestedArmPosition + armBuffer > armLength_clicks && requestedArmPosition - armBuffer < armLength_clicks){
  _armExtend.set(0.0);
  return true;
}
else if (requestedArmPosition > armLength_clicks){
  _armExtend.set(armSpeed);
}
else if (requestedArmPosition < armLength_clicks){
  _armExtend.set(-1 * armSpeed);
}
return false;  

}
  



  public void simulationPeriodic(){
    sim_armExtend.setBusVoltage(RobotController.getBatteryVoltage());
    m_armSim.setInput(sim_armExtend.getMotorOutputLeadVoltage());

    m_armSim.update(0.02);
    sim_armExtend.setIntegratedSensorRawPosition(this.distanceToNativeUnits(m_armSim.getPositionMeters()));
    SmartDashboard.putNumber("Arm Length", m_armSim.getPositionMeters());
  }

  @Override
  public void periodic() {
      if (Constants.verbose && Robot.isReal()) {
        SmartDashboard.putNumber("Arm Length",this.getArmLength());
      }
      SmartDashboard.putNumber("CoM", this.ArmComCalc());
      SmartDashboard.putNumber("Arm Clicks", _armExtend.getSelectedSensorPosition());
  }

  private int distanceToNativeUnits(double positionMeters){
    double drumRotations = positionMeters/(2 * Math.PI * 0.02);
    double motorRotations = drumRotations * 12;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }
}
