// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//Import all of the commands and subsystems. This is where you declare them.
import frc.robot.commands.*;
import frc.robot.subsystems.*;

//Various libraries that add functionality to the robot.
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  
  //Controllers and buttons. Buttons can be mapped using the DriversStation
  private XboxController controller = new XboxController(0);
  private JoystickButton controller_A = new JoystickButton(controller, 1);
  private JoystickButton controller_B = new JoystickButton(controller, 2);
  private JoystickButton controller_X = new JoystickButton(controller, 3);
  private JoystickButton controller_Y = new JoystickButton(controller, 4);
  private JoystickButton controller_leftbumper = new JoystickButton(controller, 5);
  private JoystickButton controller_rightbumper = new JoystickButton(controller, 6);
  //private JoystickButton controller_leftstickbutton = new JoystickButton(controller, 9);
  //private JoystickButton controller_rightstickbutton = new JoystickButton(controller, 10);
  private Joystick joystickA = new Joystick(1);
  private Joystick joystickB = new Joystick(2);
  private JoystickButton joystickA_6 = new JoystickButton(joystickA, 6);
  private JoystickButton joystickA_7 = new JoystickButton(joystickA, 7);
  private JoystickButton joystickA_8 = new JoystickButton(joystickA, 8);
  private JoystickButton joystickA_9 = new JoystickButton(joystickA, 9);
  private JoystickButton joystickA_10 = new JoystickButton(joystickA, 10);
  private JoystickButton joystickA_11 = new JoystickButton(joystickA, 11);
  private JoystickButton joystickA_3 = new JoystickButton(joystickA, 3);
  private JoystickButton joystickA_4 = new JoystickButton(joystickA, 4);
  private JoystickButton joystickA_5 = new JoystickButton(joystickA, 5);
  private JoystickButton joystickB_7 = new JoystickButton(joystickB, 7);
;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //The SmartDashboard is a handy tool to display and store variables.
    SmartDashboard.putNumber("Shooter Speed", 0);
    SmartDashboard.putNumber("AutoShootPower", Constants.Auto_Shooter_Speed);
    //Some subsystems have default commands. The () -> denotes a continous supply of data from 
    // the referenced value. Usually a joystick, but can be a constant.
    m_drivetrainSubsystem.setDefaultCommand(new DriveCommand(m_drivetrainSubsystem, 
                                                            () -> joystickA.getY(),
                                                            () -> joystickB.getY()));
    m_intakeSubsystem.setDefaultCommand(new IntakeAngleCommand(m_intakeSubsystem, 
                                                              () -> controller.getLeftY (), 
                                                              () -> controller.getXButton()));
    // Method to configure the buttons to perform commands.
    configureButtonBindings();
  }

  /***
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //There is unique button logic that one should consider when assigning buttons.
    // Commands that drive a motor need a constant supply of new values. This 
    // is a safety feature to make sure the robot is doing what you want it to, but can
    // be annoying to deal with.
    controller_A.whileHeld(new IndexCommand(m_intakeSubsystem, () -> 0.4));
    controller_B.whileHeld(new IndexCommand(m_intakeSubsystem, () -> -0.4));
    //controller_Y.whileHeld(new IntakePIDCommand(m_intakeSubsystem, () -> Constants.IntakePID_UpSetpoint));
    controller_Y.whileHeld(new IntakePIDCommand(m_intakeSubsystem, () -> Constants.IntakePID_IntakeSetpoint));

    //controller_leftbumper.whileHeld(new IntakeIndexCommand(m_intakeSubsystem, () -> -0.5, () -> -1.0));
    controller_X.whileHeld(new IntakeIndexCommand(m_intakeSubsystem, () -> 0.5, () -> 1.0));

    controller_rightbumper.whenHeld(new ShootCommand_Analog(m_shooterSubsystem, () -> 0.5));
    controller_leftbumper.whenHeld(new ShootCommand_Analog(m_shooterSubsystem, () -> 0.94));

    joystickA_6.whenPressed(new ClimbRotateCommand(m_climberSubsystem, () -> 1));
    joystickA_6.whenReleased(new ClimbRotateCommand(m_climberSubsystem, () -> 0));
    joystickA_7.whenPressed(new ClimbRotateCommand(m_climberSubsystem, () -> -1));
    joystickA_7.whenReleased(new ClimbRotateCommand(m_climberSubsystem, () -> 0));

    joystickA_8.whenPressed(new ClimbWinchCommand(m_climberSubsystem, () -> 0.5));
    joystickA_8.whenReleased(new ClimbWinchCommand(m_climberSubsystem, () -> 0));
    joystickA_9.whenPressed(new ClimbWinchCommand(m_climberSubsystem, () -> -0.5));
    joystickA_9.whenReleased(new ClimbWinchCommand(m_climberSubsystem, () -> 0));
    joystickB_7.whenPressed(new ResetEncodersCommand(m_climberSubsystem));
    joystickA_10.whenPressed(new Auto_FullShoot_CommandGroup(m_intakeSubsystem, m_shooterSubsystem, 0.94));
    joystickA_11.whenPressed(new Auto_FullShoot_CommandGroup(m_intakeSubsystem, m_shooterSubsystem, 0.5));
    //joystickA_3.whenPressed(new SwitchVideoCommand(m_cameraSubsystem));
    joystickA_3.whenPressed(new WinchPIDCommand(m_climberSubsystem, () -> 1.0, 0.0));
    joystickA_4.whenPressed(new WinchPIDCommand(m_climberSubsystem, () -> 1.0, 293069.0));
    joystickA_5.whenPressed(new WinchPIDCommand(m_climberSubsystem, () -> 1.0, 356685.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory) {
    // An ExampleCommand will run in autonomous
    /*return new ParallelCommandGroup(new ParallelRaceGroup(
      new Auto_Index_Command(m_intakeSubsystem, 0.3, 4.0, 2.0),
       new Auto_Shoot_Command(m_shooterSubsystem, 30000.0)),
     new Auto_Intake_Lower(m_intakeSubsystem));*/

    //This autonomus command does not use trajectories, but was able to hit a two ball automomous score.
    return new SequentialCommandGroup(
      new Auto_FullShoot_CommandGroup(m_intakeSubsystem, m_shooterSubsystem, 0.5),
      new Auto_Drive_Command(m_drivetrainSubsystem, -0.5, 0.2),
      new Auto_Index_Command(m_intakeSubsystem, 0.0, 0.5, 0.0),
      new ParallelRaceGroup(
        new Auto_Drive_Command(m_drivetrainSubsystem, -0.5, 3.0),
        new IntakeIndexCommand(m_intakeSubsystem, () -> 0.5, () -> 0.8)
      ),
      new ParallelRaceGroup(
        new IntakeIndexCommand(m_intakeSubsystem, () -> 0.0, () -> 0.0),
        new Auto_Drive_Command(m_drivetrainSubsystem, 0.0, 0.0)
      ),
      new Auto_Index_Command(m_intakeSubsystem, -0.35, 0.1, 0.0),
      new Auto_FullShoot_CommandGroup(m_intakeSubsystem, m_shooterSubsystem, 0.94)
      );
  }
}