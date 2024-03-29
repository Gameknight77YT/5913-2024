// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonSRX elevatorFollowerRight = new TalonSRX(Constants.elevatorRightID);
  private TalonSRX elevatorMasterLeft = new TalonSRX(Constants.elevatorLeftID);

  private double setpoint = 0;
  private boolean joystickControl;

  private PIDController elevatorController = new PIDController(.025, 0.0, 0.0);

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorFollowerRight.configFactoryDefault();
    elevatorMasterLeft.configFactoryDefault();
    
    elevatorFollowerRight.clearStickyFaults();
    elevatorMasterLeft.clearStickyFaults();

    //elevatorFollowerRight.follow(elevatorMasterLeft);
    elevatorMasterLeft.follow(elevatorFollowerRight);
    //elevatorMasterLeft.setInverted(true);
    //elevatorFollowerRight.setInverted(InvertType.FollowMaster);
    elevatorFollowerRight.setInverted(true);
    elevatorMasterLeft.setInverted(InvertType.FollowMaster);
    elevatorMasterLeft.setNeutralMode(NeutralMode.Brake);
    elevatorFollowerRight.setNeutralMode(NeutralMode.Brake);

    /*elevatorMasterLeft.config_kF(0, 0.0, 50); //FIXME
    elevatorMasterLeft.config_kP(0, 0.0, 50);
    elevatorMasterLeft.config_kI(0, 0.01, 50);
    elevatorMasterLeft.config_kD(0, 0.0, 50);*/
    elevatorMasterLeft.configVoltageCompSaturation(12);
    elevatorMasterLeft.enableCurrentLimit(false);
    /*elevatorMasterLeft.selectProfileSlot(0, 0);

    elevatorFollowerRight.config_kF(0, 0.0, 50);  //FIXME
    elevatorFollowerRight.config_kP(0, 0.01, 50);
    elevatorFollowerRight.config_kI(0, 0.0, 50);
    elevatorFollowerRight.config_kD(0, 0.0, 50);*/
    elevatorFollowerRight.configVoltageCompSaturation(12);
    elevatorFollowerRight.enableCurrentLimit(false);
    //elevatorFollowerRight.selectProfileSlot(0, 0);

    elevatorMasterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    elevatorFollowerRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    elevatorMasterLeft.setSensorPhase(true);
    elevatorFollowerRight.setSensorPhase(true);

    elevatorMasterLeft.setSelectedSensorPosition(0);
    elevatorFollowerRight.setSelectedSensorPosition(0);
    elevatorController.setTolerance(100);
  }

  @Override
  public void periodic() {
    if (!joystickControl) {
      double pid = elevatorController.calculate(getElevatorEncoder(), setpoint);
      pid = MathUtil.applyDeadband(pid, .4);
      elevatorFollowerRight.set(ControlMode.PercentOutput, pid);
    }
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right elevator encoder", 
      elevatorMasterLeft.getSelectedSensorPosition());

    SmartDashboard.putNumber("Left elevator encoder", 
      elevatorFollowerRight.getSelectedSensorPosition());
  }
  public void holdElevatorAtTop() {
    setElevator(-8900);
  }


  public void holdElevatorAtAmp() {
    setElevator(-4700);
  }

  

  public void setElevator(double setpoint) {
    this.setpoint = setpoint;
    
  }

  public void setElevatorJoystick(double speedElevator) {
    speedElevator = MathUtil.applyDeadband(speedElevator, .1);
    if (Math.abs(speedElevator) > .1) {
      elevatorFollowerRight.set(ControlMode.PercentOutput, MathUtil.applyDeadband(speedElevator, .1));
      joystickControl = true;
    } else if (joystickControl) {
      setpoint = getElevatorEncoder();
      joystickControl = false;
    }
  }

  public double getElevatorEncoder() {
    return elevatorFollowerRight.getSelectedSensorPosition();
  }

  public void stopElevator() {
    elevatorFollowerRight.set(ControlMode.PercentOutput, 0);
  }
}
