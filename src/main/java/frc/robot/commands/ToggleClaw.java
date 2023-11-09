package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class ToggleClaw extends CommandBase {
    private Intake m_intake;
    private DoubleSupplier outSpeed;
    private DoubleSupplier inSpeed;
    private boolean intake;

    public ToggleClaw(Intake _intake){
        m_intake = _intake;
        addRequirements(_intake);
    }
    
    @Override
        public void initialize() {
            m_intake.openGrasp();
            
           
            //SmartDashboard.putNumber("Intake Speed", inSpeed.getAsDouble());
        }
    /*@Override
    public void end(boolean interrupted) {
        m_intake.spinIntake(0);
        if (inSpeed.getAsDouble() > outSpeed.getAsDouble()) {
             SmartDashboard.putBoolean("Have Piece", true);
        } else {
            SmartDashboard.putBoolean("Have Piece", false);
        }
    }*/
    @Override
    public boolean isFinished() {
    
        return true;
}
}
