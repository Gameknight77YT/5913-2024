// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22); // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23); // Measure and set wheelbase


  //  Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = TunerConstants.kSpeedAt12VoltsMps;
          /*(6380.0 / 60.0 *
          (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * //SdsModuleConfigurations.MK4_L1.getDriveReduction()
          0.10033 * Math.PI); //SdsModuleConfigurations.MK4_L1.getWheelDiameter()*/

  public static final double MAX_acceleration_METERS_PER_SECOND = 
          MAX_VELOCITY_METERS_PER_SECOND/2;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0)/2;

          public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public static final double kPXYController = 1.0;
    public static final double kPThetaController = 1.0;
    public static final double kDThetaController = 0.5;


    //controller ids
    public static final int DriverControllerPort = 0;
    public static final int manipulatorControllerPort = 1;

    //Motor Ids 
    public static final int intakeTopID = 13;
    public static final int intakeBottomID = 14;
    public static final int shooterDeflectionID = 15;
    public static final int climberID = 16;
    public static final int shooterTopID = 17;
    public static final int shooterBottomID = 18;
    public static final int feederID = 19;
    public static final int elevatorID = 20;
    public static final int pivotID = 22;

    //other IDs 
    public static final int phID = 23;
    public static final int pdhID = 24;
    public static final int pivotEncoderID = 25;
    public static final int candleID = 26;
    
    public static final int revEncoderDIOPort = 1;

    //Solenoid ids
    public static final int climberPistonBackwardID = 15;
    public static final int climberPistonForwardID = 14;
    public static final int ampPistonForwardID = 1;
    public static final int ampPistonBackwardID = 0;

    //speeds
    public static final double intakeSpeed = 1.00; 
    public static final double feederSpeed = 1.00; 
    public static final double shooterTopShootSpeed = 75; //FIXME
    public static final double shooterBottomShootSpeed = 75; //FIXME
    public static final double shooterTopAmpShootSpeed = 10; //FIXME
    public static final double shooterBottomAmpShootSpeed = 10; //FIXME
    public static final double deflectionSpeed = .50;

    


}
