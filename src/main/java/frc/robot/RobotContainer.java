// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AmpPiston;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
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
  public final Climber climber = new Climber();
  public final Shooter shooter = new Shooter();
  public final Feeder feeder = new Feeder();
  public final Pivot pivot = new Pivot();
  private final Elevator elevator = new Elevator();
  private final AmpPiston ampPiston = new AmpPiston();
  private final ClimberPiston climberPiston = new ClimberPiston();
  private final Lights lights = new Lights(shooter, feeder);
  
  private SlewRateLimiter xLimiter = new SlewRateLimiter(18.0);//9
  private SlewRateLimiter yLimiter = new SlewRateLimiter(18.0);//9
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(35.0);//20

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.DriverControllerPort);

  private final CommandXboxController manipulatorController = 
      new CommandXboxController(Constants.manipulatorControllerPort);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.05).withRotationalDeadband(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.FieldCentric driveTracking = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.025) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  
  
  //private final Telemetry logger = new Telemetry(Constants.MAX_VELOCITY_METERS_PER_SECOND);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    
    
    // Register Named Commands
    //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
    NamedCommands.registerCommand("Track", drivetrain.applyRequest(
          () -> driveTracking
            .withRotationalRate(camera.moveInput()))
          .ignoringDisable(true)
        .alongWith(pivot.run(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance()), camera.hasValidTarget().getAsBoolean())))
        );

    NamedCommands.registerCommand("TrackOnlyPivot", (pivot.run(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance())))
        ));

    NamedCommands.registerCommand("LowerPivot", (pivot.run(
          () -> pivot.setPivotBelowStage())
        ));

    NamedCommands.registerCommand("Pivot89", pivot.run(
          () -> pivot.setPivot(85.6))
          //.alongWith(drivetrain.applyRequest(
          //  () -> driveTracking.withRotationalRate(camera.moveInput())
          //))
        );

    NamedCommands.registerCommand("Pivot88", pivot.run(
          () -> pivot.setPivot(85.6))
          //.alongWith(drivetrain.applyRequest(
          //  () -> driveTracking.withRotationalRate(camera.moveInput())
          //))
        );
    NamedCommands.registerCommand("AutoShot", pivot.run(
          () -> pivot.setPivot(86.4))
          //.alongWith(drivetrain.applyRequest(
          //  () -> driveTracking.withRotationalRate(camera.moveInput())
          //))
        );
    NamedCommands.registerCommand("AutoFirstShot", pivot.run(
          () -> pivot.setPivot(108.0))
          //.alongWith(drivetrain.applyRequest(
          //  () -> driveTracking.withRotationalRate(camera.moveInput())
          //))
        );  
      NamedCommands.registerCommand("Pivot110", pivot.run(
          () -> pivot.setPivot(115.0))
          //.alongWith(drivetrain.applyRequest(
          //  () -> driveTracking.withRotationalRate(camera.moveInput())
          //))
        );
    
    NamedCommands.registerCommand("Shoot", shooter.runEnd(() -> shooter.shoot(false),
          () -> shooter.stopShooter()));

    NamedCommands.registerCommand("Feed", shooter.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed()));

    NamedCommands.registerCommand("FeedAndShoot", shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter())
      .alongWith(feeder.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed()))
      );

    NamedCommands.registerCommand("IntakeAndSpinShoot", intake.runEnd(() -> intake.controlIntake(true), () -> intake.intakeStop())
      .alongWith(shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter())
      .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed()))
      ));

    NamedCommands.registerCommand("IntakePivotAndSpinShoot", intake.runEnd(() -> intake.controlIntake(true), () -> intake.intakeStop())
          .alongWith(shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter()))
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed())
          ));

    NamedCommands.registerCommand("IntakeBackAndSpinShoot", intake.runEnd(() -> intake.controlIntake(false), () -> intake.intakeStop())
          .alongWith(shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter()))
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed())
          ));

    NamedCommands.registerCommand("IntakeBackAndShoot", intake.runEnd(() -> intake.controlIntake(false), () -> intake.intakeStop())
          .alongWith(shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter()))
          .alongWith(feeder.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed())
          ));

    NamedCommands.registerCommand("IntakeBackPivotAndSpinShoot", intake.runEnd(() -> intake.controlIntake(false), () -> intake.intakeStop())
          .alongWith(shooter.runEnd(() -> shooter.shoot(false), () -> shooter.stopShooter()))
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed())
          ));

    

    NamedCommands.registerCommand("IntakeBack", (intake.runEnd(() -> intake.controlIntake(false), () -> intake.intakeStop())
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed())))
          .until(() -> feeder.getBeamBreak())
          );

    NamedCommands.registerCommand("IntakeForward", (intake.runEnd(() -> intake.controlIntake(true), () -> intake.intakeStop())
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed())))
          .until(() -> feeder.getBeamBreak())
          );

    NamedCommands.registerCommand("SubloaferShot", pivot.run(() -> pivot.subloaferShot()));

    //autoChooser = AutoBuilder.buildAutoChooser("2 piece");
    // Another option that allows you to specify the default auto by its name
     autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);


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
    //sysid  https://github.com/jasondaming/ctre_swerve/tree/master

    /*driverController.x().and(driverController.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    driverController.x().and(driverController.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    driverController.y().and(driverController.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    driverController.y().and(driverController.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    driverController.a().and(driverController.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    driverController.a().and(driverController.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    driverController.b().and(driverController.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    driverController.b().and(driverController.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
    */

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
          () -> drive.withVelocityX(xLimiter.calculate(-MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * Constants.MAX_VELOCITY_METERS_PER_SECOND)) // Drive forward with negative Y (forward)
            .withVelocityY(yLimiter.calculate(-MathUtil.applyDeadband(driverController.getLeftX(), .1) * Constants.MAX_VELOCITY_METERS_PER_SECOND)) // Drive left with negative X (left)
            .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(driverController.getRightX(), .1) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)) // Drive counterclockwise with negative X (right)
        ).ignoringDisable(true));

    

    // reset the field-centric heading on left bumper press FIXME set button number
    driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    //drivetrain.registerTelemetry(logger::telemeterize);

    driverController.leftTrigger().whileTrue(
        drivetrain.applyRequest(
          () -> driveTracking.withVelocityX(xLimiter.calculate(-MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * Constants.MAX_VELOCITY_METERS_PER_SECOND)) // Drive forward with negative Y (forward)
            .withVelocityY(yLimiter.calculate(-MathUtil.applyDeadband(driverController.getLeftX(), .1) * Constants.MAX_VELOCITY_METERS_PER_SECOND)) // Drive left with negative X (left)
            .withRotationalRate(camera.moveInput() + rotLimiter.calculate(-MathUtil.applyDeadband(driverController.getRightX(), .1) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)))
          .ignoringDisable(true)
        .alongWith(pivot.run(
          () -> pivot.setPivot(pivot.interpolateAngle(camera.getDistance()), camera.hasValidTarget().getAsBoolean())))
    );


    driverController.rightBumper()
      .and(driverController.rightTrigger().negate())
        .whileTrue(intake.runEnd(() -> intake.controlIntake(false, feeder.isfeedStopped()), () -> intake.intakeStop())
          .alongWith(feeder.runEnd(() -> feeder.feed(true), () -> feeder.stopFeed()))
        );

    
    
    driverController.rightBumper().negate()
      .and(driverController.rightTrigger())
        .whileTrue(intake.runEnd(() -> intake.reverseIntake(false), () -> intake.intakeStop())
          .alongWith(feeder.runEnd(() -> feeder.reverseFeed(), () -> feeder.stopFeed()))
        );

      

    
    driverController.a().whileTrue(intake.runEnd(
      () -> intake.reverseIntake(true),
      () -> intake.reverseIntake(false))
      );
    
    driverController.b().whileTrue(pivot.runOnce(() -> pivot.subloaferShot()));
    
    driverController.x().whileTrue(climber.runEnd(
      () -> climber.setClimberSpeed(.75),
      () -> climber.stopClimber())
      );
    
    driverController.y().whileTrue(climber.runEnd(
      () -> climber.setClimberSpeed(-.75),
      () -> climber.stopClimber())
      );

   // driverController.a().onTrue(climberPiston.runOnce(() -> climberPiston.toggle()));

    driverController.leftBumper()
    .and(driverController.rightBumper().negate())
    .and(driverController.rightTrigger().negate())
    .and(driverController.leftTrigger().negate())
    .whileTrue(
      pivot.runOnce(() -> pivot.setPivotBelowStage())
      .alongWith(elevator.runOnce(() -> elevator.setElevator(0)))
      .alongWith(ampPiston.runOnce(() -> ampPiston.set(false)))
      ); 

    //driverController.povLeft().onTrue(lights.runOnce(() -> lights.decrementAnimation()).ignoringDisable(true));

    //driverController.povRight().onTrue(lights.runOnce(() -> lights.incrementAnimation()).ignoringDisable(true));


    manipulatorController.rightBumper()
      .and(manipulatorController.rightTrigger().negate())
        .whileTrue(feeder.runEnd(
          () -> feeder.reverseFeed(), 
          () -> feeder.stopFeed())
        );

    manipulatorController.leftTrigger()
    .and(manipulatorController.rightTrigger().negate())
    .and(ampPiston.getPistonPose()).whileTrue(
      shooter.runEnd(() -> shooter.shoot(
          true),
          () -> shooter.stopShooter())
    );

    manipulatorController.leftTrigger()
    .and(manipulatorController.rightTrigger().negate())
    .and(ampPiston.getPistonPose().negate()).whileTrue(
      shooter.runEnd(() -> shooter.shoot(
          false),
          () -> shooter.stopShooter())
    );

    manipulatorController.leftTrigger()
    .and(manipulatorController.rightTrigger())
    .and(ampPiston.getPistonPose())
      .whileTrue(shooter.runEnd(() -> shooter.shoot(
          true),
          () -> shooter.stopShooter())
      .alongWith(feeder.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed()))
      );

    manipulatorController.leftTrigger()
    .and(manipulatorController.rightTrigger())
    .and(ampPiston.getPistonPose().negate())
      .whileTrue(shooter.runEnd(() -> shooter.shoot(
          false),
          () -> shooter.stopShooter())
          .alongWith(feeder.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed()))
      );

    manipulatorController.leftBumper().onTrue(ampPiston.runOnce(() -> ampPiston.toggle()));

    

    manipulatorController.rightTrigger().and(manipulatorController.leftTrigger().negate()).whileTrue(
        feeder.runEnd(() -> feeder.feed(false), () -> feeder.stopFeed())
        .alongWith(intake.runEnd(() -> intake.controlIntake(false, feeder.isfeedStopped()), () -> intake.intakeStop()))
      );

    manipulatorController.b().onTrue(
      pivot.runOnce(() -> pivot.topPivot())
      .alongWith(elevator.runOnce(() -> elevator.setElevator(-50)))
      .alongWith(ampPiston.runOnce(() -> ampPiston.set(true))
        )
    );

    manipulatorController.y().onTrue(
      elevator.runOnce(() -> elevator.holdElevatorAtTrap())
      .alongWith(pivot.runOnce(() -> pivot.trapPivot()))
      .alongWith(ampPiston.runOnce(() -> ampPiston.set(true))
        )
      );

    manipulatorController.x().onTrue(
      elevator.runOnce(() -> elevator.holdElevatorAtAmp())
      .alongWith(pivot.runOnce(() -> pivot.ampPivot()))
      .alongWith(ampPiston.runOnce(() -> ampPiston.set(true))
        )
    );

    manipulatorController.a().onTrue(climberPiston.runOnce(() -> climberPiston.toggle()));

    manipulatorController.povDown().onTrue(
      elevator.runOnce(() -> elevator.setElevator(0))
      .alongWith(ampPiston.runOnce(() -> ampPiston.set(false)))
    );

    

    

    

    

    
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
    return autoChooser.getSelected();
  }

  

}
