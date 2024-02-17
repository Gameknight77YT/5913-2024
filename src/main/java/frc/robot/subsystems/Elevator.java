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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private TalonSRX elevatorRight = new TalonSRX(Constants.elevatorRightID);
  private TalonSRX elevatorLeft = new TalonSRX(Constants.elevatorLeftID);
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeft.configFactoryDefault();
    elevatorRight.configFactoryDefault();
    
    elevatorLeft.clearStickyFaults();
    elevatorRight.clearStickyFaults();

    elevatorLeft.follow(elevatorRight);
    elevatorRight.setInverted(true);
    elevatorLeft.setInverted(InvertType.FollowMaster);
    elevatorRight.setNeutralMode(NeutralMode.Brake);
    elevatorLeft.setNeutralMode(NeutralMode.Brake);

    elevatorRight.config_kF(0, 0.0, 50); //FIXME
    elevatorRight.config_kP(0, 0.1, 50);
    elevatorRight.config_kI(0, 0.0, 50);
    elevatorRight.config_kD(0, 0.0, 50);
    elevatorRight.configVoltageCompSaturation(12);
    elevatorRight.enableCurrentLimit(true);
    elevatorRight.selectProfileSlot(0, 0);

    elevatorLeft.config_kF(0, 0.0, 50);  //FIXME
    elevatorLeft.config_kP(0, 0.1, 50);
    elevatorLeft.config_kI(0, 0.0, 50);
    elevatorLeft.config_kD(0, 0.0, 50);
    elevatorLeft.configVoltageCompSaturation(12);
    elevatorLeft.enableCurrentLimit(true);
    elevatorLeft.selectProfileSlot(0, 0);

    elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    elevatorLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder", 
      elevatorRight.getSelectedSensorPosition());
  }
  public void holdElevatorAtbottom() {
    elevatorRight.set(ControlMode.Position, 0);
  }

  

  public void setElevator(double setpoint) {
    elevatorRight.set(ControlMode.Position, setpoint);
  }

  public void setElevatorJoystick(double speedElevator) {
    elevatorRight.set(ControlMode.PercentOutput, MathUtil.applyDeadband(speedElevator, .1));
  }

  public void stopElevator() {
    elevatorRight.set(ControlMode.PercentOutput, 0);
  }
}
