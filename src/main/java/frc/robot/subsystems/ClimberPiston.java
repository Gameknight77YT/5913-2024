// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberPiston extends SubsystemBase {

  private DoubleSolenoid climberPistons = new DoubleSolenoid(Constants.phID, PneumaticsModuleType.REVPH, 
      Constants.climberPistonForwardID, Constants.climberPistonBackwardID);
  /** Creates a new ClimberPiston. */
  public ClimberPiston() {
    climberPistons.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(boolean out) {
    if (out) {
      climberPistons.set(Value.kForward);
    } else {
      climberPistons.set(Value.kReverse);
    }
  }

  public void toggle() {
    climberPistons.toggle();
  }


}
