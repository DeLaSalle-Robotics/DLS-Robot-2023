package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtend;

public class ArmLengthSet extends CommandBase{
    double Kp;
    double Ki;
    double Kd;
    double target;
    double error;
    double prevErr;
    double sumErr;
    ArmExtend m_armExt;

    public ArmLengthSet(double _target,ArmExtend _ArmEx){
        Kp = 1;
        Ki = 0;
        Kd = 0;
        target = _target;
        m_armExt = _ArmEx;
    }
    public void initialize(){
        error = target - m_armExt.getArmLength();
        prevErr = 0;
        sumErr = 0;
    }
    public void execute(){
        double Perr = Kp * error;
        double Derr = Kd * (error - prevErr)/0.02;
        sumErr = sumErr + error * 0.02;
        double Ierr = Ki * sumErr;
        double output = Perr + Derr + Ierr;
        if (output > 0.8){
            m_armExt.ArmExtention(0.8);
        } else if (output < -0.8){
            m_armExt.ArmExtention(-0.8);
        } else {
            m_armExt.ArmExtention(output);
        }
        SmartDashboard.putNumber("CoM", m_armExt.ArmComCalc());
        SmartDashboard.putNumber("Arm Length", m_armExt.getArmLength());
    }
    @Override
    public boolean isFinished() {
        return Math.abs(error) < 0.05;
    }
}
