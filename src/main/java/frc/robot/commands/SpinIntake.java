package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class SpinIntake extends CommandBase {
    private Intake m_intake;

    public SpinIntake(Intake _intake){
        m_intake = _intake;
            
    }
    
    @Override
        public void execute() {
            m_intake.spinIntake(Constants.IntakeSpeed);
        }

    @Override
    public boolean isFinished() {
        return m_intake.getIntakeCurrent() > Constants.intakeCurrentThreshold;
    }
}
