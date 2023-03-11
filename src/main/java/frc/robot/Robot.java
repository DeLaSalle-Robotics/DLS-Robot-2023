//Start with our robot package
package frc.robot;

//If we want to read files and deal with errors (useful for autonomus code), we need these libraries.
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

// This was never used and caused compilation errors, so it's disabled for now
// import org.apache.commons.io.output.ThresholdingOutputStream;

import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalOutput; //This class allows digital signals to be set out, useful to communicate with ardurino
import edu.wpi.first.wpilibj.DriverStation; //This class is necessary to control the robot.


//Trajectory classes could be useful, but not fully implimented here.
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

//Main robot class
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//This class brings basic commands methods, including the ability to run them in parallel
import edu.wpi.first.wpilibj2.command.Command; 
//Command Scheduler decides what will be done and when.
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand; //creates the autonomous Command (see below)

  private RobotContainer m_robotContainer;

  // Tracks the current focus for targetting
  String currentFocus = "C2";
  // Table to hold all keys
  //List[] targettingKeys = {"R1", "R2", "R3", "C1", "C2", "C3", "L1", "L2", "L3"};
//  List<String> targettingKeys = Arrays.asList("R1", "R2", "R3", "C1", "C2", "C3", "L1", "L2", "L3");
  HashMap<String,Double[]> targettingKeys;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //0arduinoOutput.set(true);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    setupTargetting();
    // Booleans for targetting nodes
    for(String key : targettingKeys.keySet()){
      if (key == currentFocus){
        SmartDashboard.putBoolean(key, true);
      } else {
        SmartDashboard.putBoolean(key, false);
      }
    }
    
    //This would be a good place to put the targetting code.
    SmartDashboard.putBoolean("R1", false);
    SmartDashboard.putBoolean("R2", false);
    SmartDashboard.putBoolean("R3", false);
    SmartDashboard.putBoolean("C1", false);
    SmartDashboard.putBoolean("C2", false);
    SmartDashboard.putBoolean("C3", false);
    SmartDashboard.putBoolean("L1", false);
    SmartDashboard.putBoolean("L2", false);
    SmartDashboard.putBoolean("L3", false);
    SmartDashboard.putString("Current Target", currentFocus);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // Called every 20 miliseconds.
    CommandScheduler.getInstance().run();

    // Check if a new boolean is activated
    for(String key : targettingKeys.keySet()){

      // Check if a boolean is true that does not match the current focus
      if(SmartDashboard.getBoolean(key, false) && key != currentFocus){
        currentFocus = key;

        // Change all other booleans to false as a failsafe
        for (String key2 : targettingKeys.keySet()){
          if (key2 != currentFocus){
            SmartDashboard.putBoolean(key2, false);
          }
        }
        // Break the loop as all other values should be false by now, so it won't find anything that's true
        break;
      }
    }
    SmartDashboard.putString("Current Target", currentFocus);

    // Failsafe to re-activate the current key if it got turned off
    if (!(SmartDashboard.getBoolean(currentFocus, false))){
      SmartDashboard.putBoolean(currentFocus, true);
    }

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //One job of the robotcontainer is to set the autonomous Command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // If there is an autonomous command, schedule it.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

 

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void setupTargetting() {
    targettingKeys=new HashMap<>();
    targettingKeys.put("R1",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("R2",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("R3",(new Double[]{1.0,2.0,3.0}));

    targettingKeys.put("C1",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("C2",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("C3",(new Double[]{1.0,2.0,3.0}));

    targettingKeys.put("L1",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("L2",(new Double[]{1.0,2.0,3.0}));
    targettingKeys.put("L3",(new Double[]{1.0,2.0,3.0}));

  }

public void postTargetData(String target){

  Double[] data=targettingKeys.get(target);
  if (data!=null) {
    SmartDashboard.putNumber("Rotation", data[0] );
    SmartDashboard.putNumber("Pitch",data[1] );
    SmartDashboard.putNumber("Length",data[2] );
  }
}


}
