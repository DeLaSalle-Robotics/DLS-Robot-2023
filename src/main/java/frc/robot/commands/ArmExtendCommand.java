package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmExtendCommand extends CommandBase{
    Arm m_arm;
    double speed;
    public ArmExtendCommand(double _speed, Arm _arm){
        m_arm = _arm;
        speed= _speed;
    }

    @Override
    public void execute() {
        m_arm.ArmExtend(speed);
    }
}
