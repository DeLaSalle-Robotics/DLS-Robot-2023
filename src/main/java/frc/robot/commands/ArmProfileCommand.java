package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmProfileCommand extends ProfiledPIDCommand {
    double armLength;
    double armLengthKp;
    Arm m_Arm;
    boolean armLengthTest;
    public ArmProfileCommand(double targetAngleDegrees, double _armLength, Arm _Arm) {
        
        //We will likely need another ProfiledPIDController to length the Arm - simple fix for now is to pass it into the command.
        
        super(new ProfiledPIDController(
            Constants.ArmKp, 
            Constants.ArmKi, 
            Constants.ArmKd,
            new TrapezoidProfile.Constraints(
                Constants.ArmMaxRotVel,
                Constants.ArmMaxRotAccel)),
            _Arm::ArmAngle, 
            targetAngleDegrees, 
            (output ,setpoint) -> _Arm.ArmMoveVolts(output), 
            _Arm);

        getController().
        setTolerance(Math.toRadians(Constants.angleTolerance));
        armLength = _armLength;
        
        if (!Preferences.containsKey("Length Kp")) {
            Preferences.setDouble("Length Kp", armLengthKp);
          }
        m_Arm = _Arm;
    }
@Override
public void initialize() {
    SmartDashboard.putNumber("Arm Goal", m_controller.getGoal().position);
}
    @Override
    public void execute() {
        double armLengthError = armLength - m_Arm.getArmLength();
        m_Arm.ArmExtend(armLengthError * armLengthKp);
        armLengthTest = Math.abs(armLengthError) < 0.01;
        
        SmartDashboard.putNumber("Arm Feedforward", m_Arm.getFeedForward(m_Arm.ArmAngle(),m_Arm.getArmLength()));
        SmartDashboard.putNumber("Arm Angle", m_Arm.ArmAngle());
        
    }
    @Override
    public boolean isFinished() {
        return getController().atGoal();// && armLengthTest;
    }
}
