//Start with our robot package
package frc.robot;

// Unused imports
//import java.nio.file.Path;
//import java.util.Arrays;
//import java.util.List;
//import java.io.IOException;
//import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.DigitalOutput; //This class allows digital signals to be set out, useful to communicate with ardurino
//import edu.wpi.first.wpilibj.DriverStation; //This class is necessary to control the robot.
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//If we want to read files and deal with errors (useful for autonomus code), we need these libraries.
import java.util.HashMap;

// This was never used and caused compilation errors, so it's disabled for now
// import org.apache.commons.io.output.ThresholdingOutputStream;

import edu.wpi.first.wpilibj.DataLogManager;


//Trajectory classes could be useful, but not fully implimented here.
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  String currentFocus = "Mid Cone";
  // Table to hold all keys
  //List[] targettingKeys = {"R1", "R2", "R3", "C1", "C2", "C3", "L1", "L2", "L3"};
//  List<String> targettingKeys = Arrays.asList("R1", "R2", "R3", "C1", "C2", "C3", "L1", "L2", "L3");
  HashMap<String,Double[]> targetingKeys;
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
    setupTargeting();
    // Booleans for targetting nodes
    // for(String key : targetingKeys.keySet()){
    //   if (key == currentFocus){
    //     SmartDashboard.putBoolean(key, true);
    //   } else {
    //     SmartDashboard.putBoolean(key, false);
    //   }
    //}
    
    //This would be a good place to put the targetting code.
    // SmartDashboard.putString("Current Target", currentFocus);
    // SmartDashboard.putString("Score Type", "Mid");
    // SmartDashboard.putString("Piece Type", "Cube");
    
    // SmartDashboard.putBoolean("Cone Low", false);
    // SmartDashboard.putBoolean("Cone Mid", false);
    // SmartDashboard.putBoolean("Cone High", false);
    // SmartDashboard.putBoolean("Cube Low", false);
    // SmartDashboard.putBoolean("Cube Mid", false);
    // SmartDashboard.putBoolean("Cube High", false);
    
    // SmartDashboard.putBoolean("Arm Calibration", false);
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
     currentFocus = String.format("%s %s", 
                                SmartDashboard.getString("Piece Type","Cube"),
                                SmartDashboard.getString("Score Type", "Mid"));

    // Check if a new boolean is activated
    // for(String key : targetingKeys.keySet()){

    //   // Check if a boolean is true that does not match the current focus
    //   if(SmartDashboard.getBoolean(key, false) && key == currentFocus){
    //     SmartDashboard.putBoolean(key, true); 
    //   } else{
    //     // Change all other booleans to false as a failsafe
    //         SmartDashboard.putBoolean(key, false);
    //       }
    //     }
    //     // Break the loop as all other values should be false by now, so it won't find anything that's true
        //break;
    
    //SmartDashboard.putString("Current Target", currentFocus);

    // Failsafe to re-activate the current key if it got turned off
    // if (!(SmartDashboard.getBoolean(currentFocus, false))){
    //   SmartDashboard.putBoolean(currentFocus, true);
    // }


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
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    

// get a topic from a NetworkTableInstance
// the topic name in this case is the full name
    BooleanTopic AllianceTopic = inst.getBooleanTopic("/FMSInfo/IsRedAlliance");
    boolean allBool = AllianceTopic.subscribe(false).get();
    SmartDashboard.putBoolean("Red Alliance", allBool);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    this.postTargetData(SmartDashboard.getString("Current Target", "Cube Mid"));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void setupTargeting() {
    targetingKeys=new HashMap<>();
    //Targeting values are: arm angle, arm length, arm angle (back score) <-- need to check
    targetingKeys.put("Cone High",(new Double[]{38.4,1.8,141.6}));
    targetingKeys.put("Cone Mid",(new Double[]{38.4,1.37,141.6}));
    targetingKeys.put("Cone Low",(new Double[]{0.0,0.3,180.0}));

    targetingKeys.put("Cube High",(new Double[]{33.6,1.47, 146.4}));
    targetingKeys.put("Cube Mid",(new Double[]{33.6,0.95, 146.4}));
    targetingKeys.put("Cube Low",(new Double[]{0.0,0.3, 180.0}));

  }

public void postTargetData(String target){

  Double[] data=targetingKeys.get(target);
  
  boolean scoreFront = SmartDashboard.getBoolean("Score Front", true);
  if (data!=null) {
    
    if (scoreFront) {
       SmartDashboard.putNumber("Pitch",data[0] );
     } else {
       SmartDashboard.putNumber("Pitch", data[2]);
     }
     SmartDashboard.putNumber("Length",data[1] );
   }
  boolean loadFront= SmartDashboard.getBoolean("Load Front", false);
  //Setting load angles <--Need to be confirmed
  if (loadFront){
    SmartDashboard.putNumber("Load Angle", 45);
  } else {
     SmartDashboard.putNumber("Load Angle", 135);
   }
}


public void cancelAll(){
  CommandScheduler.getInstance().cancelAll();
}



}
