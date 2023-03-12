// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private Color mode;

  public Leds() {
     // PWM port 9?
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
  //change the length later!
    m_ledBuffer = new AddressableLEDBuffer(12);
    m_led.setLength(m_ledBuffer.getLength());


    // Set the data
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for cube
      m_ledBuffer.setRGB(i, 148, 0, 211);
   }
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() {
     // Set the LEDs
     m_led.setData(m_ledBuffer);
  }

    // This method will be called once per scheduler run
     // Fill the buffer with a rainbow
     // Set the LEDs
   
   public void orange() {
     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
     // Sets the specified LED to the RGB values for cone
       m_ledBuffer.setRGB(i, 255, 100, 0);
      }
  }
 
  public void purple() {
   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
     // Sets the specified LED to the RGB values for cube
     m_ledBuffer.setRGB(i, 148, 0, 211);
  }
  }
}
