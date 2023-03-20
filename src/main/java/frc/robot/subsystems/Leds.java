// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.WristConstants.IntakeConstants;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private final Intake intake;
  private final Drivetrain drivetrain;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private final int NUMBER_OF_LEDS = 80;
  private final int EDGE_GAP = 0;// DO NOT DISPLAY THE SPEED
  /** the number of leds including the lit led */
  private final int MOVING_LED_GAP = 5;
  private boolean goingUp = false;
  private Color centerColor;
  private double centerMultiplier = 1.0;
  private ArrayList<Integer> movingLights = new ArrayList<Integer>();
  private int timer = 0;
  // private Color mode;

  public Leds(Intake intake, Drivetrain drivetrain) {
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
    centerColor = new Color(255, 255, 255);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for cube
      // m_ledBuffer.setRGB(i, 148, 0, 211);
      m_ledBuffer.setLED(i, centerColor);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();

    this.drivetrain = drivetrain;

    for (int i = 0; i < m_ledBuffer.getLength() / MOVING_LED_GAP; i++) {
      movingLights.add(i * MOVING_LED_GAP);
    }

  }

  @Override
  public void periodic() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    if (DriverStation.isEnabled() && DriverStation.isTeleop() && inst.isConnected()) {
      // Set the LEDs
      if (intake.getIntakeSpeed() >= -IntakeConstants.INTAKE_SPEED_DEADBAND
          && intake.getIntakeSpeed() <= IntakeConstants.INTAKE_SPEED_DEADBAND) {
        if (goingUp && centerMultiplier >= 1.0) {
          goingUp = false;
        }
        if (!goingUp && centerMultiplier <= 0.1) {
          goingUp = true;
        }

        if (goingUp) {
          centerMultiplier += 0.05;
        } else {
          centerMultiplier -= 0.05;
        }

      } else {
        centerMultiplier = 1.0;
        goingUp = false;
      }
      Color centerColorWithMultiplier = new Color(centerColor.red * centerMultiplier,
          centerColor.green * centerMultiplier, centerColor.blue * centerMultiplier);
      for (var i = EDGE_GAP; i < m_ledBuffer.getLength() - EDGE_GAP; i++) {
        m_ledBuffer.setLED(i, centerColorWithMultiplier);
      }

      setSpeed(drivetrain.getMaxDrivetrainSpeed());
    } else if (inst.isConnected()){
      if (timer % 5 == 0) {
        autoColor();
      }
      timer++;
    }
    else{
      centerColor = new Color(0, 255, 0);
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, centerColor);
      }
    }
    m_led.setData(m_ledBuffer);

  }


  public void orange() {
    centerColor = new Color(255, 100, 0);
  }

  public void purple() {
    centerColor = new Color(148, 0, 211);
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

  private void autoColor() {

    // increase them all by one
    for (int i = 0; i < movingLights.size(); i++) {
      movingLights.set(i, movingLights.get(i) + 1);
    }
    // if the last one is over, subtract the number of leds
    if (movingLights.get(movingLights.size() - 1) >= m_ledBuffer.getLength()) {
      movingLights.set(movingLights.size() - 1, movingLights.get(movingLights.size() - 1) - m_ledBuffer.getLength());
      movingLights.sort(null);
    }

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (movingLights.contains(i)) {
        m_ledBuffer.setLED(i, new Color(255, 255, 255));
      } else {
        if (DriverStation.getAlliance() == Alliance.Blue) {
          m_ledBuffer.setLED(i, new Color(0, 0, 255));
        } else {
          m_ledBuffer.setLED(i, new Color(255, 0, 0));

        }

      }
    }

  }
}
