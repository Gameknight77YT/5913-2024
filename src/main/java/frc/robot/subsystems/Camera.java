// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Camera extends SubsystemBase {

  public static final String limeLightName = "limelight";

  private PIDController pidController = new PIDController(0.1, 0., 0.0);
  
  /** Creates a new Camera. */
  public Camera() {

  }

  public double getDistance() {
    LimelightHelpers.setPipelineIndex(limeLightName, 2);
    double a1 = Units.degreesToRadians(30);
    double a2 = Units.degreesToRadians(LimelightHelpers.getTY(limeLightName));
    double h1 = LimelightHelpers.getCameraPose3d_RobotSpace(limeLightName).getZ();
    double h2 = Units.inchesToMeters(51.875 + 10.25);
    double dis = (h2-h1) / Math.tan(a1+a2);
    //double x = LimelightHelpers.getTargetPose3d_RobotSpace(limeLightName).getX();
    //double y = LimelightHelpers.getTargetPose3d_RobotSpace(limeLightName).getY();
    //double dis = Math.sqrt(x*x + y*y);
    return dis;
  }

  public Trigger hasValidTarget() { 
    var latestResults = LimelightHelpers.getLatestResults(limeLightName);
    BooleanSupplier booleanSupplier = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return latestResults.targetingResults.valid;
      }
    };

    return new Trigger(booleanSupplier);
  }

  public double moveInput() {
    if(!hasValidTarget().getAsBoolean()){
      return 0;
    }else{
      double tx = LimelightHelpers.getTX(limeLightName);
      return pidController.calculate(tx, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("distance", getDistance());
    SmartDashboard.putNumber("move input", moveInput());
  }
}
