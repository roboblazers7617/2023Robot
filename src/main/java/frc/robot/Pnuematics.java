// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ArmConstants.PnuematicsConstants;

public class Pnuematics{
  private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private DoubleSolenoid leftArmPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PnuematicsConstants.LEFT_ARM_PISTON_EXTEND_PORT, PnuematicsConstants.LEFT_ARM_PISTON_RETRACT_PORT);
  private DoubleSolenoid rightArmPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PnuematicsConstants.RIGHT_ARM_PISTON_EXTEND_PORT, PnuematicsConstants.RIGHT_ARM_PISTON_RETRACT_PORT);
  /** Creates a new Pnuematics. */
  public Pnuematics() {

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
