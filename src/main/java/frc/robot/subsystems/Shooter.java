// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterTop = new TalonFX(Constants.shooterTopID);
  private TalonFX shooterBottom = new TalonFX(Constants.shooterBottomID);

  private TalonFXConfiguration cfg1 = new TalonFXConfiguration();
  private TalonFXConfiguration cfg2 = new TalonFXConfiguration();
  
  private final VelocityVoltage shooterTopVelocityVoltage = new VelocityVoltage(0);
  private final VelocityVoltage shooterBottomVelocityVoltage = new VelocityVoltage(0);

  private TalonSRX feeder = new TalonSRX(Constants.feederID);

  private DigitalInput beamBreak = new DigitalInput(0);

  private Timer intakeTimer = new Timer();

  /*TODO: private DoubleSolenoid ampPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.ampPistonForwardID, Constants.ampPistonBackwardID);*/

  
  
  /** Creates a new Shooter. */
  public Shooter() {
    cfg1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg1.Slot0.kV = 0.0;   //FIXME
    cfg1.Slot0.kP = 0.65;
    cfg1.Slot0.kI = 0.0;
    cfg1.Slot0.kD = 0.01;

    cfg2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg2.Slot0.kV = 0.0;   //FIXME
    cfg2.Slot0.kP = 0.65;
    cfg2.Slot0.kI = 0.0;
    cfg2.Slot0.kD = 0.01;

    

    feeder.configFactoryDefault();
    

    shooterTop.clearStickyFaults();
    shooterBottom.clearStickyFaults();
    feeder.clearStickyFaults();

    shooterTop.getConfigurator().apply(cfg1);
    shooterBottom.getConfigurator().apply(cfg2);

    shooterTopVelocityVoltage.Slot = 0;
    shooterBottomVelocityVoltage.Slot = 0;

    feeder.setNeutralMode(NeutralMode.Coast);

    //TODO: ampPistons.set(Value.kReverse);

    intakeTimer.reset();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter top speed", 
      shooterTop.getVelocity().getValue());
    
    SmartDashboard.putNumber("commanded speed", shooterTopVelocityVoltage.Velocity);

    SmartDashboard.putNumber("shooter bottom speed", 
      shooterBottom.getVelocity().getValue());

    SmartDashboard.putBoolean("Beam Break", beamBreak.get());

    SmartDashboard.putNumber("timer", intakeTimer.get());
    
  }

  

  public void shoot(boolean isAmp) {
    if (isAmp) {
      //ampPistons.set(Value.kForward);
      shooterTop.setControl(shooterTopVelocityVoltage.withVelocity(Constants.shooterTopAmpShootSpeed));
      shooterBottom.setControl(shooterBottomVelocityVoltage.withVelocity(Constants.shooterBottomAmpShootSpeed));
    } else {
      //ampPistons.set(Value.kReverse);
      shooterTop.setControl(shooterTopVelocityVoltage.withVelocity(Constants.shooterTopShootSpeed));
      shooterBottom.setControl(shooterBottomVelocityVoltage.withVelocity(Constants.shooterBottomShootSpeed));
    }
  }

  public void feedAndShoot(boolean isAmp) {
    shoot(isAmp);
    feed(false);
  }

  public void stopFeedAndShoot() {
    stopShooter();
    stopFeed();
  }

  

  public void feed(boolean intake) {
    if (intake) {
      if (!beamBreak.get()) {
        intakeTimer.start();
        feeder.setNeutralMode(NeutralMode.Brake);
        feeder.set(ControlMode.PercentOutput, 0);

      } else if (intakeTimer.get() > 0.0 && intakeTimer.get() < 3) {
        feeder.set(ControlMode.PercentOutput, 0);
        feeder.setNeutralMode(NeutralMode.Brake);

      } else if (intakeTimer.get() > 3) {
        intakeTimer.stop();
        intakeTimer.reset();
      
      } else {
        
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

  public void stopShooter() {
    shooterTop.set(0);
    shooterBottom.set(0);
  }

  public void stopFeed() {
    feeder.set(ControlMode.PercentOutput, 0);
  }

  

}
