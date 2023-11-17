package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class SpinIntake extends CommandBase {
    private Intake m_intake;
    private DoubleSupplier outSpeed;
    private DoubleSupplier inSpeed;
    private boolean intake;
    Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);

    public SpinIntake(Intake _intake, DoubleSupplier _outSpeed, DoubleSupplier _inSpeed){
        
        m_intake = _intake;
        outSpeed = _outSpeed;
        inSpeed = _inSpeed; 
        addRequirements(_intake);
    }
    
    @Override
    public void initialize(){
        debounce.calculate(false);
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
        if (debounce.calculate(m_intake.getIntakeCurrent() > 15.0)){
            m_intake.closeGrasp();
            m_intake.stopIntake();
            SmartDashboard.putBoolean("Have Piece", true);
            return true;
        } else {
            return false;
        }
    
    }
}
