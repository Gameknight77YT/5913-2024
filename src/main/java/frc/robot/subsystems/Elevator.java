// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private double setpoint = 0;
  private boolean joystickControl;

  private TalonFX elevator = new TalonFX(Constants.elevatorID);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController elevatorController = new PIDController(.08, 0.00, 0.001);

  /** Creates a new Elevator. */
  public Elevator() {

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    
    elevator.getConfigurator().apply(cfg);

    //elevator.setPosition(0);
    
    elevatorController.setTolerance(100);
  }

  @Override
  public void periodic() {
    if (!joystickControl) {
      double pid = elevatorController.calculate(getElevatorEncoder(), setpoint);
      pid = MathUtil.applyDeadband(pid, .4);
      elevator.set(pid);
    }
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder", 
      getElevatorEncoder());

  }
  public void holdElevatorAtTrap() {
    setElevator(-190);
  }


  public void holdElevatorAtAmp() {
    setElevator(-102);
  }

  

  public void setElevator(double setpoint) {
    this.setpoint = setpoint;
    
  }

  public void setElevatorJoystick(double speedElevator) {
    speedElevator = MathUtil.applyDeadband(speedElevator, .1);
    if (Math.abs(speedElevator) > .1) {
      elevator.set(MathUtil.applyDeadband(speedElevator, .1));
      joystickControl = true;
    } else if (joystickControl) {
      setpoint = getElevatorEncoder();
      joystickControl = false;
    }
  }

  public double getElevatorEncoder() {
    return elevator.getPosition().getValueAsDouble();
  }

  public void stopElevator() {
    elevator.set(0);
  }
}
