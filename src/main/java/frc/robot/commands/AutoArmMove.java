package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;

public class AutoArmMove extends CommandBase{
    DoubleSupplier speed;
    ArmExtend m_armExtend;
    public AutoArmMove(DoubleSupplier _speed, ArmExtend _armExtend){
        speed = _speed;
        m_armExtend = _armExtend;
        addRequirements(_armExtend);
    }
    @Override
    public void execute() {
        m_armExtend.ArmExtention(speed.getAsDouble());
        if (speed.getAsDouble() > 0.3) {
        SmartDashboard.putBoolean("Compressor", false);
        } else {
            SmartDashboard.putBoolean("Compressor", true);
        }
    }
    
}
