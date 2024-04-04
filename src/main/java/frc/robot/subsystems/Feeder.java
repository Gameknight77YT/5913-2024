// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  private DigitalInput beamBreak = new DigitalInput(0);

  private boolean intakeStopped = false;

  private Timer intakeTimer = new Timer();

  private TalonSRX feeder = new TalonSRX(Constants.feederID);
  /** Creates a new Feeder. */
  public Feeder() {
    feeder.configFactoryDefault();
    feeder.clearStickyFaults();
    feeder.setNeutralMode(NeutralMode.Coast);

    intakeTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Beam Break", getBeamBreak());

    SmartDashboard.putNumber("timer", intakeTimer.get());
  }

  public void stopFeed() {
    intakeStopped = false;
    feeder.set(ControlMode.PercentOutput, 0);
  }

  public void feed(boolean intake) {
    if (intake) {
      if (getBeamBreak()) {
        
        intakeTimer.start();
        feeder.setNeutralMode(NeutralMode.Brake);
        feeder.set(ControlMode.PercentOutput, 0);
      

      } else if (intakeTimer.get() > 0.00 && intakeTimer.get() < 3) {
        if(intakeTimer.get() > .2) {
          intakeStopped = true;
        }
        feeder.set(ControlMode.PercentOutput, 0);
        feeder.setNeutralMode(NeutralMode.Brake);
        

      } else if (intakeTimer.get() > 3) {
        intakeTimer.stop();
        intakeTimer.reset();
      
      } else {
        intakeStopped = false;
        feeder.set(ControlMode.PercentOutput, Constants.feederSpeed * .7);
        feeder.setNeutralMode(NeutralMode.Coast);

      }
    } else {
      feeder.set(ControlMode.PercentOutput, Constants.feederSpeed);
    }
  }

  public void reverseFeed() {
    feeder.set(ControlMode.PercentOutput, -Constants.feederSpeed);
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  public boolean isfeedStopped() {
    return intakeStopped || getBeamBreak();
  }
}
