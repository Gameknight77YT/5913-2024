// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControlIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Camera camera = new Camera();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  public final Intake intake = new Intake();
  //TODO: public final Climber climber = new Climber();
  public final Shooter shooter = new Shooter();
  public final Pivot pivot = new Pivot();
  private final Elevator elevator = new Elevator();
  //public final Compressor compressor = new Compressor();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DriverControllerPort);

  private final CommandXboxController manipulatorController = 
      new CommandXboxController(Constants.manipulatorControllerPort);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  //private final Telemetry logger = new Telemetry(Constants.MAX_VELOCITY_METERS_PER_SECOND);

  //private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    
    
    // Register Named Commands
    //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
    NamedCommands.registerCommand("Track", (drivetrain.applyRequest(
          () -> forwardStraight.withRotationalRate(camera.moveInput()))
        .alongWith(pivot.runEnd(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance())), 
          () -> pivot.stopPivot()))
        ));

    NamedCommands.registerCommand("TrackOnlyPivot", (pivot.runEnd(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance())), 
          () -> pivot.stopPivot())
        ));
    
    NamedCommands.registerCommand("Shoot", shooter.runEnd(() -> shooter.shoot(false),
          () -> shooter.stopShooter()));

    NamedCommands.registerCommand("Feed", shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed()));

    NamedCommands.registerCommand("FeedAndShoot", shooter.runEnd(() -> shooter.feedAndShoot(false), () -> shooter.stopFeedAndShoot()));

    NamedCommands.registerCommand("IntakeAndShoot", new ControlIntake(intake, true)
          .alongWith(shooter.runEnd(() -> shooter.feedAndShoot(false), () -> shooter.stopFeedAndShoot()))
          //.alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
          );

    NamedCommands.registerCommand("IntakeBackAndShoot", new ControlIntake(intake, false)
          .alongWith(shooter.runEnd(() -> shooter.feedAndShoot(false), () -> shooter.stopFeedAndShoot()))
          //.alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
          );

    NamedCommands.registerCommand("IntakeBack", new ControlIntake(intake, false)
          .alongWith(shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed()))
          .alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
          );

    NamedCommands.registerCommand("IntakeForward", new ControlIntake(intake, true)
          .alongWith(shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed()))
          .alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
          );

    NamedCommands.registerCommand("SubloaferShot", pivot.runEnd(() -> pivot.subloaferShot(), () -> pivot.stopPivot()));

    //autoChooser = AutoBuilder.buildAutoChooser("2 piece");
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    //SmartDashboard.putData("Auto Chooser", autoChooser);


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> drive.withVelocityX(-MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(driverController.getLeftX(), .1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), .1) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (right)
        ).ignoringDisable(true));

    //driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //driverController.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press FIXME set button number
    driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    //drivetrain.registerTelemetry(logger::telemeterize);
    

    //driverController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //driverController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    driverController.leftTrigger().whileTrue(
        drivetrain.applyRequest(
          () -> forwardStraight.withVelocityX(-MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(driverController.getLeftX(), .1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
            .withRotationalRate(camera.moveInput()))
          .ignoringDisable(true)
        .alongWith(pivot.runEnd(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance())), 
          () -> pivot.stopPivot()))
    );




    driverController.rightBumper().negate()
      .and(driverController.rightTrigger())
        .whileTrue(new ControlIntake(intake, true)
        .alongWith(shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed()))
        .alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
      );
    
    driverController.rightBumper()
      .and(driverController.rightTrigger().negate())
        .whileTrue(new ControlIntake(intake, false)
          .alongWith(shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed()))
          .alongWith(pivot.runEnd(() -> pivot.setPivotForIntake(), () -> pivot.stopPivot()))
        );

    /*driverController.pov(0).whileTrue(climber.runEnd(
      () -> climber.setClimberSetpoint(Constants.climberBottomSetpoint), 
      () -> climber.stopClimber()));
    
    driverController.pov(180).whileTrue(climber.runEnd(
      () -> climber.setClimberSetpoint(Constants.climberTopSetpoint), 
      () -> climber.stopClimber()));
      
      hold button
      */

    /*driverController.pov(0).onTrue(climber.run(
      () -> climber.setClimberSetpoint(Constants.climberBottomSetpoint))
      .onlyWhile(climber.isNotAtSetpoint()));
    
    driverController.pov(180).onTrue(climber.run(
      () -> climber.setClimberSetpoint(Constants.climberTopSetpoint))
      .onlyWhile(climber.isNotAtSetpoint()));
    */

    manipulatorController.rightBumper()
      .and(manipulatorController.rightTrigger().negate())
        .whileTrue(shooter.runEnd(
          () -> shooter.reverseFeed(), 
          () -> shooter.stopFeed())
        );

    manipulatorController.leftTrigger().and(manipulatorController.rightTrigger().negate()).whileTrue(
        shooter.runEnd(() -> shooter.shoot(
          manipulatorController.leftBumper().getAsBoolean()),
          () -> shooter.stopShooter()));

    manipulatorController.rightTrigger().and(manipulatorController.leftTrigger().negate()).whileTrue(
        shooter.runEnd(() -> shooter.feed(), () -> shooter.stopFeed())
      );

    manipulatorController.leftTrigger().and(manipulatorController.rightTrigger())
      .whileTrue(shooter.runEnd(() -> shooter.feedAndShoot(
          manipulatorController.leftBumper().getAsBoolean()),
          () -> shooter.stopFeedAndShoot()));

    /*manipulatorController.a().whileTrue(shooter.runEnd(
      () -> shooter.setElevator(Constants.elevatorBottomSetpoint), 
      () -> shooter.stopElevator()));
    
    manipulatorController.y().whileTrue(shooter.runEnd(
      () -> shooter.setElevator(Constants.elevatorTopSetpoint), 
      () -> shooter.stopElevator()));*/

    /*manipulatorController.b().whileTrue(elevator.run(
        () -> elevator.setElevatorJoystick(manipulatorController.getRightY()))
      ).whileFalse(elevator.runEnd(
        () -> elevator.holdElevatorAtbottom(), 
        () -> elevator.stopElevator())
      );*/

    

    
    elevator.setDefaultCommand(elevator.run(
        () -> elevator.setElevatorJoystick(manipulatorController.getRightY()))
      );
    
    pivot.setDefaultCommand(pivot.run(
        () -> pivot.setPivotJoystick((manipulatorController.getLeftY())))
      );


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoBuilder.buildAuto("2 piece");
  }

  

}
