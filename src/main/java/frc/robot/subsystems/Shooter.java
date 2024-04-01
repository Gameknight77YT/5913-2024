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
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterTop = new TalonFX(Constants.shooterTopID);
  private TalonFX shooterBottom = new TalonFX(Constants.shooterBottomID);
  private CANSparkMax shooterDeflection = new CANSparkMax(Constants.shooterDeflectionID, CANSparkLowLevel.MotorType.kBrushless);

  private TalonFXConfiguration cfg1 = new TalonFXConfiguration();
  private TalonFXConfiguration cfg2 = new TalonFXConfiguration();
  
  private final VelocityVoltage shooterTopVelocityVoltage = new VelocityVoltage(0);
  private final VelocityVoltage shooterBottomVelocityVoltage = new VelocityVoltage(0);

  

  
  
  /** Creates a new Shooter. */
  public Shooter() {
    cfg1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg1.Slot0.kV = 0.0;   //FIXME
    cfg1.Slot0.kP = 0.7;
    cfg1.Slot0.kI = 0.0;
    cfg1.Slot0.kD = 0.01;
    //cfg1.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    cfg2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg2.Slot0.kV = 0.0;   //FIXME
    cfg2.Slot0.kP = 0.7;
    cfg2.Slot0.kI = 0.0;
    cfg2.Slot0.kD = 0.01;
    //cfg2.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    

    
    shooterDeflection.restoreFactoryDefaults();
    

    shooterTop.clearStickyFaults();
    shooterBottom.clearStickyFaults();
    
    shooterDeflection.clearFaults();

    shooterTop.getConfigurator().apply(cfg1);
    shooterBottom.getConfigurator().apply(cfg2);

    shooterTopVelocityVoltage.Slot = 0;
    shooterBottomVelocityVoltage.Slot = 0;

    

    
    shooterDeflection.setIdleMode(IdleMode.kCoast);

    shooterDeflection.setInverted(false);

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter top speed", 
      shooterTop.getVelocity().getValue());
    
    SmartDashboard.putNumber("commanded speed", shooterTopVelocityVoltage.Velocity);

    SmartDashboard.putNumber("shooter bottom speed", 
      shooterBottom.getVelocity().getValue());

    

    SmartDashboard.putBoolean("shooter up to speed", isShooterUpToSpeed());
    
  }

  public boolean isShooterUpToSpeed() {
    return shooterTop.getVelocity().getValueAsDouble() > 60;
  }

  

  public void shoot(boolean isAmp) {
    if (isAmp) {
      shooterDeflection.set(Constants.deflectionSpeed);
      shooterTop.setControl(shooterTopVelocityVoltage.withVelocity(Constants.shooterTopAmpShootSpeed));
      shooterBottom.setControl(shooterBottomVelocityVoltage.withVelocity(Constants.shooterBottomAmpShootSpeed));
    } else {
      shooterDeflection.set(0);
      shooterTop.setControl(shooterTopVelocityVoltage.withVelocity(Constants.shooterTopShootSpeed));
      shooterBottom.setControl(shooterBottomVelocityVoltage.withVelocity(Constants.shooterBottomShootSpeed));
    }
  }

  

  

  

  

  

  public void stopShooter() {
    shooterTop.set(0);
    shooterBottom.set(0);
    shooterDeflection.set(0);
  }

  

  

}
