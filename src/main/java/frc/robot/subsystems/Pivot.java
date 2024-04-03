// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interpolating.InterpolatingDouble;
import frc.robot.subsystems.Interpolating.InterpolatingTreeMap;

public class Pivot extends SubsystemBase {
  private TalonFX pivot = new TalonFX(Constants.pivotID);

  //private CANcoder pivotEncoder = new CANcoder(Constants.pivotEncoderID);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.revEncoderDIOPort);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();
  //private CANcoderConfiguration cfg2 = new CANcoderConfiguration();

  private double pivotSetpoint;
  private boolean joystickControl;

  //private final PositionVoltage pivotPositionVoltage = new PositionVoltage(0);

  private PIDController pivotController = new PIDController(0.2, 0.00, 0.0);

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  /** Creates a new Pivot. */
  public Pivot() {
    /*cfg2.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cfg2.MagnetSensor.MagnetOffset = Units.degreesToRotations(-114+13);
    cfg2.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoder.getConfigurator().apply(cfg2);*/

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //cfg.Slot0.kP = 0.4;   //not here scroll up
    //cfg.Slot0.kI = 0.001;
    //cfg.Slot0.kD = 0.00;
    //cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    /*cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .2;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .2;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;*/

    //pivotEncoder.setPositionOffset(Units.degreesToRotations(270));
    new Thread(() -> {
      try {
          Thread.sleep(2000);
          pivotSetpoint = getPivotAngleDegrees();
      } catch (Exception e) {
      }
  }).start();
    
    
    pivot.clearStickyFaults();
    
    pivot.getConfigurator().apply(cfg);
    
    //pivotPositionVoltage.Slot = 0;

    //put(4.15, 74.0);
    //put(3.98, 74.0);
    //put(3.86, 78.72);
    //put(3.74, 79.69);
    //put(3.57, 81.1);
    put(10.0, 85.20);
    put(3.9, 85.20);
    put(3.43, 89.7);
    put(3.31, 90.7);
    put(3.01, 93.3);
    put(2.71, 95.5);
    put(2.36, 100.77);
    put(1.93, 109.5);

    pivotController.setTolerance(.5);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    
    if (!joystickControl) {
      double pid = -pivotController.calculate(getPivotAngleDegrees(), pivotSetpoint);
      pid = MathUtil.applyDeadband(pid, .05);
      pivot.set(pid);
    }
    
    
    SmartDashboard.putNumber("pivot encoder", 
      pivot.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("pivot encoder abs", getPivotAngleDegrees());

    SmartDashboard.putNumber("pivot target", pivotController.getSetpoint());//pivotPositionVoltage.Position);
    SmartDashboard.putNumber("error", pivotController.getPositionError());
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

  public double getPivotAngleDegrees() {
    return Units.rotationsToDegrees(-pivotEncoder.getAbsolutePosition())+270;
  }

  public void setPivot(double setpoint) {
    pivotSetpoint = setpoint;
  }

  public void setPivot(double setpoint, boolean hasValidTarget) {
    if (hasValidTarget) {
      setPivot(setpoint);
    } else {
      
    }
  }

  public void subloaferShot() {
    setPivot(105);
  }

  public double interpolateAngle(double distance) {
    return pivotMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public void setPivotJoystick(double speedPivot) {
    speedPivot = MathUtil.applyDeadband(speedPivot, .1);
    if (Math.abs(speedPivot) > .1) {
      pivot.set(speedPivot);
      joystickControl = true;
    } else if (joystickControl) {
      pivotSetpoint = getPivotAngleDegrees();
      joystickControl = false;
    }
    
  }
  
  public void setPivotBelowStage() {
    //pivot.setControl(pivotPositionVoltage.withPosition(73));
    setPivot(83.0);
  }

  //public void stopPivot() {
  //  pivot.set(0);
  //}

  public void topPivot() {
      setPivot(134);
  }

  public void ampPivot() {
      setPivot(111);
  }

  public void trapPivot() {
      setPivot(107);
  }

}
