// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
  /** Creates a new ColorSensor. */
  public ColorSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int getProximity(){
    return sensor.getProximity();
  }

  public int getBlue(){
    return sensor.getBlue();
  }

  public int getGreen(){
    return sensor.getGreen();
  }

  public int getRed(){
    return sensor.getRed();
  }
}
