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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Arm m_Arm = new Arm();
  private final Intake m_grasper = new Intake();
  private final ArmExtend m_armExtend = new ArmExtend();
  
  //Controllers and buttons. Buttons can be mapped using the DriversStation
  private XboxController controller = new XboxController(0);

  private Trigger controller_A = new JoystickButton(controller, 1);
  private Trigger controller_B = new JoystickButton(controller, 2);
  private Trigger controller_X = new JoystickButton(controller, 3);
  private Trigger controller_Y = new JoystickButton(controller, 4);
  private Trigger controller_leftbumper = new JoystickButton(controller, 5);
  private Trigger controller_rightbumper = new JoystickButton(controller, 6);
  //private Trigger controller_leftstickbutton = new JoystickButton(controller, 9);
  //private Trigger controller_rightstickbutton = new JoystickButton(controller, 10);
  private Joystick joystickA = new Joystick(1);
  private Joystick joystickB = new Joystick(2);
  private Trigger joystickA_6 = new JoystickButton(joystickA, 6);
  private Trigger joystickA_7 = new JoystickButton(joystickA, 7);
  private Trigger joystickA_8 = new JoystickButton(joystickA, 8);
  private Trigger joystickA_9 = new JoystickButton(joystickA, 9);
  private Trigger joystickA_10 = new JoystickButton(joystickA, 10);
  private Trigger joystickA_11 = new JoystickButton(joystickA, 11);
  private Trigger joystickA_3 = new JoystickButton(joystickA, 3);
  private Trigger joystickA_4 = new JoystickButton(joystickA, 4);
  private Trigger joystickA_1 = new JoystickButton(joystickA, 1);
  private Trigger joystickA_2 = new JoystickButton(joystickA, 2);

  //Defining POV buttons
  private POVButton controller_Up = new POVButton(controller, 0);
  private POVButton controller_Right = new POVButton(controller, 90);
  private POVButton controller_Down = new POVButton(controller, 180);
  private POVButton controller_Left = new POVButton(controller, 270);

  private String tajectoryJSON = "Paths/Output/output/1_Red_In.wpilib.json";

  private final Command m_red_Right_Engage = new Auto_Red_Right_Engage(m_drivetrainSubsystem, m_Arm, m_grasper, m_armExtend);
  private final Command m_red_Right_NoEngage = new Auto_Red_Right_NoEngage(m_drivetrainSubsystem);

  SendableChooser<Command> m_chooser = new SendableChooser();

//  private String tajectoryJSON = "Paths/Output/output/1_Red_to_Cone.wpilib.json";
;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //The SmartDashboard is a handy tool to display and store variables.
    //Some subsystems have default commands. The () -> denotes a continous supply of data from 
    // the referenced value. Usually a joystick, but can be a constant.
    m_drivetrainSubsystem.setDefaultCommand(new DriveCommand(m_drivetrainSubsystem, 
                                                            () -> joystickA.getY(),
                                                            () -> joystickB.getY(),
                                                            () -> joystickA_3.getAsBoolean()));
                                                            
     m_Arm.setDefaultCommand(new ArmMoveCommand( m_Arm, () -> joystickA.getX()));
    m_armExtend.setDefaultCommand(new ArmLengthDrive(() -> joystickB.getX(), m_armExtend));

    // Method to configure the buttons to perform commands.
    configureButtonBindings();
    m_chooser.setDefaultOption("Red Right Engage", m_red_Right_Engage);
    m_chooser.addOption("Red Right NoEngage", m_red_Right_NoEngage);
    SmartDashboard.putData(m_chooser);
  }

  /***
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // controller_A.onTrue(new TestCommand(m_miniArm, () -> controller.getLeftY()));
    controller_B.onTrue(Commands.runOnce(m_Arm::ResetArmEncoder, m_Arm));
    //controller_A.onTrue(Commands.runOnce(m_grasper::closeGrasp, m_grasper));
    //controller_A.onFalse(Commands.runOnce(m_grasper::openGrasp, m_grasper));
    controller_X.onTrue(new ArmPlaceCommand(180, 0.3, m_Arm,m_armExtend));
    controller_Y.onTrue(new ArmPlaceCommand(0, 1.0, m_Arm, m_armExtend));
    controller_leftbumper.whileTrue(new ArmVoltStatic(m_Arm));
    controller_rightbumper.whileTrue(new ArmVoltQuasistatic(m_Arm));
    
    joystickA_1.onTrue(new ArmPlaceCommand(180, 0.4, m_Arm,m_armExtend));
    joystickA_2.onTrue(new ArmPlaceCommand(0, 1.0, m_Arm, m_armExtend));
    joystickA_3.onTrue(new TestingPoses(m_drivetrainSubsystem));
    joystickA_4.onTrue(new ResetingPoses(m_drivetrainSubsystem));
    
    controller_Up.onTrue(new ArmLengthDrive(() -> Constants.ControlArmSpeed, m_armExtend));
    controller_Down.onTrue(new ArmLengthDrive(() -> -Constants.ControlArmSpeed, m_armExtend));
    controller_Left.onTrue(new DrivetrainControlRotate(Constants.ControlDriveSpeed, m_drivetrainSubsystem));
    controller_Right.onTrue(new DrivetrainControlRotate(-Constants.ControlDriveSpeed, m_drivetrainSubsystem));

        /*
     * It is possible to string commands together from one button press. This might be useful for the
     * intake where we engage the pneumatics after the intake wheels are stopped. Example code:
     * exampleButtion.onTrue(Commands.run(m_grasper::RunIntake, m_grasper))
     * .onFalse(Commands.run(m_grasper::CloseJaws, m_grasper));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return 
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    // An ExampleCommand will run in autonomous
    /*return new ParallelCommandGroup(new ParallelRaceGroup(
      new Auto_Index_Command(m_intakeSubsystem, 0.3, 4.0, 2.0),
       new Auto_Shoot_Command(m_shooterSubsystem, 30000.0)),
     new Auto_Intake_Lower(m_intakeSubsystem));*/

    //This autonomus command does not use trajectories, but was able to hit a two ball automomous score.
    
  }
}