// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX climber = new TalonFX(Constants.climberID);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PositionVoltage climberPositionVoltage = new PositionVoltage(0);

  /** Creates a new Climber. */
  public Climber() {
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    cfg.Slot0.kP = 0.1;  //FIXME
    cfg.Slot0.kI = 0.00;
    cfg.Slot0.kD = 0.00;  
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    climber.clearStickyFaults();

    climber.getConfigurator().apply(cfg);

    climber.setPosition(0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber encoder", climber.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public BooleanSupplier isNotAtSetpoint() {
    return new BooleanSupplier() {
      public boolean getAsBoolean() {
        return !(Math.abs(climber.getPosition().getValueAsDouble() - climberPositionVoltage.Position) < 100);
      }
      
    };
  }

  public void setClimberSetpoint(double setpoint) {
    climber.setControl(climberPositionVoltage.withPosition(setpoint));
  }

  public void setClimberSpeed(double speedPercent) {
    climber.set(speedPercent);
  }

  public void stopClimber() {
    climber.setControl(climberPositionVoltage.withPosition(climber.getPosition().getValueAsDouble()));
  }

  public void climberTop() {
    climber.setControl(climberPositionVoltage.withPosition(0));
  }

  public void climberDown() {
    climber.setControl(climberPositionVoltage.withPosition(-170));
  }
}
