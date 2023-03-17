// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private final Intake intake;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private final int NUMBER_OF_LEDS = 30;
  private final int EDGE_GAP = 4;
  private boolean goingUp = false;
  private Color centerColor;
  private double centerMultiplier = 1.0;
  // private Color mode;

  public Leds(Intake intake) {
    // PWM port 9?
    // Must be a PWM header, not MXP or DIO
    this.intake = intake;
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // change the length later!
    m_ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    centerColor = new Color(0, 255, 0);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for cube
      // m_ledBuffer.setRGB(i, 148, 0, 211);
      m_ledBuffer.setLED(i, centerColor);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() {
    // Set the LEDs
    if(intake.getIntakeSpeed() != 0 ? true: false){
      if (goingUp && centerMultiplier >= 1.0) {
        goingUp = false;
      }
      if (!goingUp && centerMultiplier <= 0.5) {
        goingUp = true;
      }

      if(goingUp){
        centerMultiplier += 0.05;
      }
      else{
        centerMultiplier -= 0.05;
      }
      
    }
    else{
      centerMultiplier = 1.0;
      goingUp = false;
    }
    Color centerColorWithMultiplier = new Color(centerColor.red * centerMultiplier, centerColor.green * centerMultiplier, centerColor.blue * centerMultiplier);
    for (var i = EDGE_GAP; i < m_ledBuffer.getLength() - EDGE_GAP; i++) {
      m_ledBuffer.setLED(i, centerColorWithMultiplier);
    }
    m_led.setData(m_ledBuffer);

  }

  // This method will be called once per scheduler run
  // Fill the buffer with a rainbow
  // Set the LEDs

  public void orange() {
    centerColor = new Color(255, 100, 0);
    
  }

  public void purple() {
    centerColor = new Color(148, 0, 211);
    // for (var i = EDGE_GAP; i < m_ledBuffer.getLength() - EDGE_GAP; i++) {
    //   // Sets the specified LED to the RGB values for cube
    //   m_ledBuffer.setLED(i, centerColor);
    // }
  }

  public void setSpeed(double speed) {
    if (speed == DrivetrainConstants.FAST_SPEED) {
      for (int i = 0; i < EDGE_GAP; i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
      for (int i = m_ledBuffer.getLength() - EDGE_GAP - 1; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
    }
    if (speed == DrivetrainConstants.REG_SPEED) {
      for (int i = 0; i < EDGE_GAP; i++) {
        m_ledBuffer.setRGB(i, 255, 119, 0);
      }
      for (int i = m_ledBuffer.getLength() - EDGE_GAP - 1; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 119, 0);
      }
    }
    if (speed == DrivetrainConstants.SLOW_SPEED) {
      for (int i = 0; i < EDGE_GAP; i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
      for (int i = m_ledBuffer.getLength() - EDGE_GAP - 1; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
    }

  }
}
