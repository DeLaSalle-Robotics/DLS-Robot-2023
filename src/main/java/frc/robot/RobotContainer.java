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
  private XboxController Tcontroller = new XboxController(4);

  private Trigger Tcontroller_A = new JoystickButton(Tcontroller, 1);
  private Trigger Tcontroller_B = new JoystickButton(Tcontroller, 2);
  private Trigger Tcontroller_X = new JoystickButton(Tcontroller, 3);
  private Trigger Tcontroller_Y = new JoystickButton(Tcontroller, 4);
  private Trigger Tcontroller_leftbumper = new JoystickButton(Tcontroller, 5);
  private Trigger Tcontroller_rightbumper = new JoystickButton(Tcontroller, 6);
  //Defining POV buttons
  private POVButton Tcontroller_Up = new POVButton(Tcontroller, 0);
  private POVButton Tcontroller_Right = new POVButton(Tcontroller, 90);
  private POVButton Tcontroller_Down = new POVButton(Tcontroller, 180);
  private POVButton Tcontroller_Left = new POVButton(Tcontroller, 270);

  private XboxController controller = new XboxController(0);

  private Trigger controller_A = new JoystickButton(controller, 1);
  private Trigger controller_B = new JoystickButton(controller, 2);
  private Trigger controller_X = new JoystickButton(controller, 3);
  private Trigger controller_Y = new JoystickButton(controller, 4);
  private Trigger controller_leftbumper = new JoystickButton(controller, 5);
  private Trigger controller_rightbumper = new JoystickButton(controller, 6);
  private Trigger controller_start = new JoystickButton(controller, 8);
  private Trigger controller_back = new JoystickButton(controller, 7);
  //Defining POV buttons
  private POVButton controller_Up = new POVButton(controller, 0);
  private POVButton controller_Right = new POVButton(controller, 90);
  private POVButton controller_Down = new POVButton(controller, 180);
  private POVButton controller_Left = new POVButton(controller, 270);

  //private Trigger controller_leftstickbutton = new JoystickButton(controller, 9);
  //private Trigger controller_rightstickbutton = new JoystickButton(controller, 10);
  private Joystick Left_joystick = new Joystick(1);
  private Joystick Right_joystick = new Joystick(2);
  private Trigger Left_joystick_1 = new JoystickButton(Left_joystick,1);
  private Trigger Right_joystick_1 = new JoystickButton(Right_joystick,1);
  
  

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
                                                            () -> Left_joystick.getX(),
                                                            () -> Right_joystick.getY()));
                                                            
    m_Arm.setDefaultCommand(new ArmMoveCommand(m_Arm, () -> controller.getLeftY()));
    m_armExtend.setDefaultCommand(new ArmLengthDrive(() -> controller.getRightY(), m_armExtend));
    m_grasper.setDefaultCommand(new SpinIntake(m_grasper, () -> controller.getLeftTriggerAxis(),
                                                          () -> controller.getRightTriggerAxis()));

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
  
    //Floor Mode
    controller_A.onTrue(new ArmPlaceCommand(
        200,
        0.3, m_Arm, m_armExtend));
    //Score Mode
    controller_X.onTrue(new ScoreArmCommand( m_Arm, m_armExtend));
    //Feeder
    controller_B.onTrue(new LoadArmCommand(m_Arm, m_armExtend));
    controller_Y.onTrue(new NeutralArmCommand(m_Arm, m_armExtend));

    controller_Up.onTrue(Commands.runOnce(m_grasper::scoreHigh));
    controller_Down.onTrue(Commands.runOnce(m_grasper::scoreLow));
    controller_Left.onTrue(Commands.runOnce(m_grasper::scoreMid));
    controller_Right.onTrue(Commands.runOnce(m_grasper::scoreMid));

    controller_leftbumper.onTrue(Commands.runOnce(m_grasper::intakeFlip));
    controller_rightbumper.onTrue(Commands.runOnce(m_grasper::openGrasp));
    controller_start.onTrue(Commands.runOnce(m_grasper::scoreCube));
    controller_back.onTrue(Commands.runOnce(m_grasper::scoreCone));

    Tcontroller_A.whileTrue(new ArmVoltStatic(m_Arm));
    Tcontroller_B.whileTrue(new ArmVoltQuasistatic(m_Arm));
    Tcontroller_X.onTrue(Commands.runOnce(m_grasper::intakeFlip));
    Tcontroller_Y.onTrue(Commands.runOnce(m_grasper::openGrasp));
    Tcontroller_leftbumper.onTrue(Commands.runOnce(m_grasper::enableCompressor));
    Tcontroller_rightbumper.onTrue(Commands.runOnce(m_grasper::disableCompressor));
    
    Tcontroller_Up.onTrue(new TrajectoryCalibrate(m_drivetrainSubsystem));
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
    
  }
}