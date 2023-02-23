// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PnuematicsConstants;

public class Pnuematics extends SubsystemBase {
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid leftArmPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PnuematicsConstants.LEFT_ARM_PISTON_EXTEND_PORT, PnuematicsConstants.LEFT_ARM_PISTON_RETRACT_PORT);
  private DoubleSolenoid rightArmPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PnuematicsConstants.RIGHT_ARM_PISTON_EXTEND_PORT, PnuematicsConstants.RIGHT_ARM_PISTON_RETRACT_PORT);
  /** Creates a new Pnuematics. */
  public Pnuematics() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enable(){
    compressor.enableDigital();
  }

  public void disable(){
    compressor.disable();
  }

  public boolean isEnabled() {
    return compressor.isEnabled();
  }

  public DoubleSolenoid getLeftArmPiston(){
    return leftArmPiston;
  }

  public DoubleSolenoid getRightArmPiston(){
    return rightArmPiston;
  }
}
