// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric drive;
  private Double x,y,r;
  private boolean isTrack;
  private double moveInput;
  /** Creates a new DriveCommand. */
  public DriveCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, double x, double y, double r, boolean isTrack, double moveInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.r = r;
    this.isTrack = isTrack;
    this.moveInput = moveInput;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(
          () -> drive.withVelocityX(-MathUtil.applyDeadband(y, 0.1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
            .withVelocityY(-MathUtil.applyDeadband(x, .1) * Constants.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
            .withRotationalRate(-MathUtil.applyDeadband(r, .1) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (right)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
