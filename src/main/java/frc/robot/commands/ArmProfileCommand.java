package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmProfileCommand extends ProfiledPIDCommand {
    double armLength;
    double armLengthKp;
    Arm m_Arm;
    boolean armLengthTest;
    public ArmProfileCommand(double targetAngleDegrees, double _armLength, Arm m_Arm) {
        //We will likely need another ProfiledPIDController to length the Arm - simple fix for now is to pass it into the command.
        
        super(new ProfiledPIDController(
            Constants.ArmKp, 
            Constants.ArmKi, 
            Constants.ArmKd,
            new TrapezoidProfile.Constraints(
                Constants.ArmMaxRotVel,
                Constants.ArmMaxRotAccel)),
            m_Arm::ArmAngle, 
            Math.toRadians(targetAngleDegrees), 
            (output ,setpoint) -> m_Arm.ArmMoveVolts(output), 
            m_Arm);

        getController().
        setTolerance(Math.toRadians(Constants.angleTolerance));
        armLength = _armLength;
        
        if (!Preferences.containsKey("Length Kp")) {
            Preferences.setDouble("Length Kp", armLengthKp);
          }
        
    }

    @Override
    public void execute() {
        double armLengthError = armLength - m_Arm.getArmLength();
        m_Arm.ArmExtend(armLengthError * armLengthKp);
        armLengthTest = Math.abs(armLengthError) < 0.01;
    }
    @Override
    public boolean isFinished() {
        return getController().atGoal() && armLengthTest;
    }
}
