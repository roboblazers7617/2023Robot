// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.LedStrip;
import frc.PredefinedColors;
import frc.Animations.BounceAnimation;

public class Leds extends LedStrip{

  public Leds() {
    super(9, 75);
    setAnimation(new BounceAnimation(this, PredefinedColors.kMaroon, PredefinedColors.kPaleVioletRed, 10));
  }
}
