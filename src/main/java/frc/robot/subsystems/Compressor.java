// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  private PneumaticHub ph = new PneumaticHub();
  private edu.wpi.first.wpilibj.Compressor compressor = new edu.wpi.first.wpilibj.Compressor(PneumaticsModuleType.REVPH);
  /** Creates a new Commpresser. */
  public Compressor() {
    compressor.enableAnalog(90, 120);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("airPressure", compressor.getPressure());
    // This method will be called once per scheduler run
  }
}
