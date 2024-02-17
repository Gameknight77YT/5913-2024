// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX intakeTop = new TalonSRX(Constants.intakeTopID);
  private TalonSRX intakeBottom = new TalonSRX(Constants.intakeBottomID);

  //private CANSparkMax intakeTopBack = new CANSparkMax(Constants.intakeTopBackID, MotorType.kBrushless);

  /*private DoubleSolenoid backIntakeArms = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    Constants.backIntakeArmsForwardID, Constants.backIntakeArmsBackwardID);*/

  /** Creates a new Intake. */
  public Intake() {
    intakeTop.configFactoryDefault();
    intakeBottom.configFactoryDefault();
    //intakeTopBack.restoreFactoryDefaults();

    intakeTop.setInverted(false);
    intakeBottom.setInverted(true);
    //intakeTopBack.setInverted(false);

    intakeTop.setNeutralMode(NeutralMode.Coast);
    intakeBottom.setNeutralMode(NeutralMode.Coast);
    //intakeTopBack.setIdleMode(IdleMode.kCoast);

    intakeTop.clearStickyFaults();
    intakeBottom.clearStickyFaults();
    //intakeTopBack.clearFaults();

    //backIntakeArms.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void controlIntake(boolean isForward) {
    if (isForward) {
      intakeFront();
      retractBackIntakeArms();
    }else{
      intakeBack();
      exstendBackIntakeArms();
    }
  }


  public void intakeFront() {
    intakeTop.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    intakeBottom.set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    //intakeTopBack.set(0);
  }
  
  public void intakeBack() {
    intakeTop.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    intakeBottom.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    //intakeTopBack.set(Constants.intakeSpeed);
  }

  public void intakeStop() {
    intakeTop.set(ControlMode.PercentOutput, 0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
    //intakeTopBack.set(0);
  }

  public void exstendBackIntakeArms() {
    //backIntakeArms.set(Value.kReverse);
  }

  public void retractBackIntakeArms() {
    //backIntakeArms.set(Value.kForward);
  }

  
}
