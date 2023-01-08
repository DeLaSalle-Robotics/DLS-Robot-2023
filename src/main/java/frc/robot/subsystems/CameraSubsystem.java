// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase { 
  UsbCamera camera1;
  UsbCamera camera2;
  NetworkTableEntry cameraSelection;
  private boolean cameraToggle;

  //Declaring the subsystem components
  public CameraSubsystem() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    cameraToggle = false;
  }
  //This is a method within the subsystem
  public void toggleCamera() {
    //Method allows the videofeed to switch between cameras.
    if (cameraToggle) {
      System.out.println("Setting Camera 1");
      cameraSelection.setString(camera1.getName());
      cameraToggle = false;
    }
    else {
      System.out.println("Setting Camera 2");
      cameraSelection.setString(camera2.getName());
      cameraToggle = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
