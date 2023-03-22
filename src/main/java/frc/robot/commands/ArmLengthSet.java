package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;

public class ArmLengthSet extends PIDCommand {
   
    public ArmLengthSet(double _target, ArmExtend _ArmEx){
        super(
            new PIDController(Constants.ArmLenKp, Constants.ArmLenKi, Constants.ArmLenKd),
             _ArmEx::getArmLength,
             _target,
              output -> _ArmEx.ArmExtentionVolts(output), 
              _ArmEx);
              //getController().enableContinuousInput(0, 1.7);
              getController().
              setTolerance(0.05);
              addRequirements(_ArmEx);
    }
    
    @Override
    public boolean isFinished() {
        //if (getController().atSetpoint()) {System.out.println("ArmLengthSet Finished");}
        return getController().atSetpoint();
    }
}
