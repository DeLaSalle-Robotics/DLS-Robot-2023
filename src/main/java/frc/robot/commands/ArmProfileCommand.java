package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmProfileCommand extends ProfiledPIDCommand {
    double target;
    double armLengthKp;
    Arm m_Arm;
    boolean armLengthTest;
    public ArmProfileCommand(double targetAngleRad, Arm _Arm) {
        
        //We will likely need another ProfiledPIDController to length the Arm - simple fix for now is to pass it into the command.
        
        super(new ProfiledPIDController(
            Constants.ArmKp, 
            Constants.ArmKi, 
            Constants.ArmKd,
            new TrapezoidProfile.Constraints(
                Constants.ArmMaxRotVel,
                Constants.ArmMaxRotAccel), 0.02),
            _Arm::ArmAngle, 
            targetAngleRad, 
            (output ,setpoint) -> _Arm.ArmMoveVolts(output.doubleValue()), 
            _Arm);

        getController().enableContinuousInput(-180, 180);
        getController().
        setTolerance(Math.toRadians(Constants.angleTolerance));
        
        
        if (!Preferences.containsKey("Length Kp")) {
            Preferences.setDouble("Length Kp", armLengthKp);
          }
        m_Arm = _Arm;
        target = targetAngleRad;

    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        SmartDashboard.putNumber("Arm Setting", getController().getPositionError());
    }
    @Override
    public boolean isFinished() {
        
        if (Math.abs(m_Arm.ArmAngle() - target) < Math.toRadians(Constants.angleTolerance)) { 
            System.out.println("Profiled Control Done");
            
    }
        return getController().atGoal();
    }
}
