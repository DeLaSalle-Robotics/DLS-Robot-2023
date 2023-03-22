package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmMoveCommand extends CommandBase{
    private final Arm m_Arm;
    DoubleSupplier speed;

    public ArmMoveCommand ( Arm _arm, DoubleSupplier _speed){
        m_Arm = _arm;
        speed = _speed;
        addRequirements(_arm);
    }

    @Override
    public void execute() {
        m_Arm.ArmMove(speed.getAsDouble());
        if (Constants.verbose) {SmartDashboard.putNumber("Manual Arm", speed.getAsDouble());}
    }
}
