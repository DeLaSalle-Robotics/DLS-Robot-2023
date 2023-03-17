package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class SpinIntake extends CommandBase {
    private Intake m_intake;

    public SpinIntake(Intake _intake){
        m_intake = _intake;
            
    }
    
    @Override
        public void execute() {
           if (SmartDashboard.getBoolean("Have Piece", false)) {
            m_intake.spinIntake(-Constants.IntakeSpeed);
            } else {
                m_intake.spinIntake(-Constants.IntakeSpeed);
            }
        }
    @Override
    public void end(boolean interrupted) {
        if (SmartDashboard.getBoolean("Have Piece", false)) {
            SmartDashboard.putBoolean("Have Piece", true);
            } else {
                SmartDashboard.putBoolean("Have Piece", false);
            }

    }
    @Override
    public boolean isFinished() {
        if(Robot.isReal()) {return m_intake.getIntakeCurrent() > Constants.intakeCurrentThreshold;
        } else {return true;}
    }
}
