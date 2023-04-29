package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class SpinIntake extends CommandBase {
    private Intake m_intake;
    private DoubleSupplier outSpeed;
    private DoubleSupplier inSpeed;
    private boolean intake;

    public SpinIntake(Intake _intake, DoubleSupplier _outSpeed, DoubleSupplier _inSpeed){
        m_intake = _intake;
        outSpeed = _outSpeed;
        inSpeed = _inSpeed; 
        addRequirements(_intake);
    }
    
    @Override
        public void execute() {
            if (inSpeed.getAsDouble() > 0) {
                m_intake.spinIntake(-1 * inSpeed.getAsDouble()); //negative pulls piece in
                SmartDashboard.putNumber("Intake Speed", inSpeed.getAsDouble());//m_intake.spinVelocity());
            } else {
                m_intake.spinIntake(outSpeed.getAsDouble() );
                SmartDashboard.putNumber("Intake Speed", outSpeed.getAsDouble());
            }
            
           
            //SmartDashboard.putNumber("Intake Speed", inSpeed.getAsDouble());
        }
    @Override
    public void end(boolean interrupted) {
        m_intake.spinIntake(0);
        if (inSpeed.getAsDouble() > outSpeed.getAsDouble()) {
             SmartDashboard.putBoolean("Have Piece", true);
        } else {
            SmartDashboard.putBoolean("Have Piece", false);
        }
    }
    @Override
    public boolean isFinished() {
    
        if(Robot.isReal()) {
            if (inSpeed.getAsDouble() > outSpeed.getAsDouble()) {

                return SmartDashboard.getNumber("Intake Speed", 0) < Constants.intakeSpinThreshold;
            } else {
                return outSpeed.getAsDouble() > 0.8 && SmartDashboard.getBoolean("Have Piece", false) ;
            }
        } else {return SmartDashboard.getBoolean("Have Piece", false);}
    
    }
}
