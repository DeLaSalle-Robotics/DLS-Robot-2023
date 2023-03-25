package frc.robot.commands;

// Unused imports
//import java.util.function.DoubleConsumer;
//import java.util.function.DoubleSupplier;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class KeepArmPosition extends PIDCommand{
    

    public KeepArmPosition (double targetAngle, Arm arm)  {
        super(
            new PIDController(Constants.ArmKp_static, Constants.ArmKi_static, Constants.ArmKd_static),
             arm::ArmAngle,
             Math.toRadians(targetAngle),
              output -> arm.ArmMoveVolts(output), 
              arm);
              getController().enableContinuousInput(-180, 180);
              getController().
              setTolerance(Math.toRadians(Constants.angleTolerance));
              addRequirements(arm);
    }
    
}
