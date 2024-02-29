// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class AmpPiston extends SubsystemBase {

  private DoubleSolenoid ampPistons = new DoubleSolenoid(Constants.phID, PneumaticsModuleType.REVPH, 
      Constants.ampPistonForwardID, Constants.ampPistonBackwardID);
  /** Creates a new AmpPiston. */
  public AmpPiston() {
    ampPistons.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(boolean out) {
    if (out) {
      ampPistons.set(Value.kForward);
    } else {
      ampPistons.set(Value.kReverse);
    }
  }

  public void toggle() {
    ampPistons.toggle();
  }

  public Trigger getPistonPose() {
    BooleanSupplier supplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return ampPistons.get() == Value.kForward;
      }
      
    };

    return new Trigger(supplier);
  }


}
