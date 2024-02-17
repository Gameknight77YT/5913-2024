// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interpolating.InterpolatingDouble;
import frc.robot.subsystems.Interpolating.InterpolatingTreeMap;

public class Pivot extends SubsystemBase {
  private TalonFX pivot = new TalonFX(Constants.pivotID);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private final PositionVoltage pivotPositionVoltage = new PositionVoltage(0);

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  /** Creates a new Pivot. */
  public Pivot() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.Slot0.kP = 0.25;   //FIXME
    cfg.Slot0.kI = 0.001;
    cfg.Slot0.kD = 0.00;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 25.0;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -230;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    
    pivot.clearStickyFaults();
    
    pivot.getConfigurator().apply(cfg);
    
    pivotPositionVoltage.Slot = 0;

    put(2.085, -49.0);
    put(2.402, -36.3);
    put(2.702, -23.04);
    put(3.010, -6.629);
    put(3.2, 0.0);
    put(5.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("pivot encoder", 
      pivot.getPosition().getValue());

    SmartDashboard.putNumber("pivot target", pivotPositionVoltage.Position);
  }

  /**
   * 
   * @param dis
   * @param pivot
   */
  private void put(Double dis, Double pivot){
    //key = distance, value = speed
    InterpolatingDouble k = new InterpolatingDouble(dis);
    InterpolatingDouble v = new InterpolatingDouble(pivot);
    pivotMap.put(k, v);  
  }

  public void setPivot(double setpoint) {
    pivot.setControl(pivotPositionVoltage.withPosition(setpoint));
  }

  public void subloaferShot() {
    setPivot(-100);
  }

  public double interpolateAngle(double distance) {
    return pivotMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public void setPivotJoystick(double speedPivot) {
    pivot.set(MathUtil.applyDeadband(speedPivot, .1));
    
  }
  
  public void setPivotForIntake() {
    pivot.setControl(pivotPositionVoltage.withPosition(20));
  }

  public void stopPivot() {
    pivot.set(0);
  }

}
