package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;

public class LoadArmCommand extends CommandBase{
    double armAngle;
    double armLength;
    Arm arm;
    ArmExtend armExtend;
    public LoadArmCommand(Arm _arm, ArmExtend _armExtend) {
        armAngle = SmartDashboard.getNumber("Load Angle", 0);
        armLength = 0.3;
        arm = _arm;
        armExtend = _armExtend;
        
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        armAngle = SmartDashboard.getNumber("Load Angle", 0);
        armLength = 0.3;
        CommandScheduler.getInstance().schedule(new ArmPlaceCommand(armAngle, armLength, arm, armExtend));
    }
    @Override
    public boolean isFinished() {
        return Math.abs(armAngle - SmartDashboard.getNumber("Current Arm Angle", 0)) < Constants.angleTolerance;
    }
    
}
