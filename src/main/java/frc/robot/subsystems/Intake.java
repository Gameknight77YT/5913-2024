// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX intakeTop = new TalonFX(Constants.intakeTopID);
  private TalonSRX intakeBottom = new TalonSRX(Constants.intakeBottomID);

 

  //private CANSparkMax intakeTopBack = new CANSparkMax(Constants.intakeTopBackID, MotorType.kBrushless);

  /*private DoubleSolenoid backIntakeArms = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
    Constants.backIntakeArmsForwardID, Constants.backIntakeArmsBackwardID);*/

    private TalonFXConfiguration cfg = new TalonFXConfiguration();

  /** Creates a new Intake. */
  public Intake() {

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //cfg.CurrentLimits.SupplyCurrentLimit = 20;
    //cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    //cfg.CurrentLimits.SupplyCurrentThreshold = 30;
    //cfg.CurrentLimits.SupplyTimeThreshold = .1;
    
    intakeBottom.configFactoryDefault();
    intakeTop.getConfigurator().apply(cfg);
    //intakeTopBack.restoreFactoryDefaults();

    intakeTop.setInverted(true);
    intakeBottom.setInverted(true);
    //intakeTopBack.setInverted(false);

    intakeTop.setNeutralMode(NeutralModeValue.Coast);
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

  public void controlIntake(boolean isForward, Boolean beamBreak) {
    if (!beamBreak) {
      controlIntake(isForward);
    } else {
      intakeStop();
    }
  }

  public void reverseIntake(boolean isForward) {
    if (isForward) {
      intakeTop.set(-Constants.intakeSpeed);
      intakeBottom.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    } else {
      intakeTop.set(-Constants.intakeSpeed);
      intakeBottom.set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    }
  }


  public void intakeFront() {
    intakeTop.set(Constants.intakeSpeed);
    intakeBottom.set(ControlMode.PercentOutput, -Constants.intakeSpeed);
    //intakeTopBack.set(0);
  }
  
  public void intakeBack() {
    intakeTop.set(Constants.intakeSpeed);
    intakeBottom.set(ControlMode.PercentOutput, Constants.intakeSpeed);
    //intakeTopBack.set(Constants.intakeSpeed);
  }

  public void intakeStop() {
    intakeTop.set(0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
    //intakeTopBack.set(0);
  }


  public void exstendBackIntakeArms() {
    //backIntakeArms.set(Value.kReverse);
  }

  public void retractBackIntakeArms() {
    //backIntakeArms.set(Value.kForward);
  }

  public void intakeReverse() {
    
  }
}
