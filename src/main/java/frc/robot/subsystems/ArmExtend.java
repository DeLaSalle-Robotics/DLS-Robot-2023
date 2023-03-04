package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
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
    4.0,7.0,0.02, 0.3683,1.397,true,VecBuilder.fill(0.01)
);

  public ArmExtend(){
    _armExtend.configFactoryDefault();
    _armExtend.setNeutralMode(NeutralMode.Brake);
    _armExtend.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);
    _armExtend.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 0, 30));
  }
  public double getArmLength(){
    if (Robot.isReal()){
    double armLength_clicks = _armExtend.getSelectedSensorPosition();
    //Conversion figure to convert length to sensor position
    double armlength_m = 0 * armLength_clicks + .3; //<-- needs to be determined.
    return armlength_m;} else {
       return m_armSim.getPositionMeters();
    }
  }

  public void ArmExtention(double speed) {
    //This method sets the speed of the arm extension motor
    _armExtend.set(speed);
  }
  
  public double ArmComCalc(){

    double Arm_Com = this.getArmLength() * 0.5711 - 0.0639; //Function to convert armlength to Center of Mass distance from pivot <- Derived from measurments.
  
    return Arm_Com;
  }

  public void simulationPeriodic(){
    sim_armExtend.setBusVoltage(RobotController.getBatteryVoltage());
    m_armSim.setInput(sim_armExtend.getMotorOutputLeadVoltage());

    m_armSim.update(0.02);
    sim_armExtend.setIntegratedSensorRawPosition(this.distanceToNativeUnits(m_armSim.getPositionMeters()));
    SmartDashboard.putNumber("Arm Length", m_armSim.getPositionMeters());
  }

  private int distanceToNativeUnits(double positionMeters){
    double drumRotations = positionMeters/(2 * Math.PI * 0.02);
    double motorRotations = drumRotations * 4;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }
}
