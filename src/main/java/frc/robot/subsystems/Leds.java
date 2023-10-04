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
    private final int NUMBER_OF_LEDS = 75;
    private final int EDGE_GAP = 0;// DO NOT DISPLAY THE SPEED
    /** the number of leds including the lit led */
    private final int MOVING_LED_GAP = 20;
    private boolean goingUp = false;
    private Color centerColor;
    private double centerMultiplier = 1.0;
    private ArrayList<Integer> movingLightsUp = new ArrayList<Integer>();
    private ArrayList<Integer> movingLightsDown = new ArrayList<Integer>();
    private int bouncePosition = 0;
    private boolean bounceGoingUp = true;
    private final int BOUNCE_LENGTH = 5;

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
            movingLightsUp.add(i * MOVING_LED_GAP);
        }
        for (int i = 0; i < m_ledBuffer.getLength() / MOVING_LED_GAP; i++) {
            movingLightsDown.add(i * MOVING_LED_GAP);
        }

    }

    @Override
    public void periodic() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        if (!DriverStation.isEStopped()) {
            if (DriverStation.isEnabled() && DriverStation.isTeleop() && inst.isConnected()) {
                // Set the LEDs
                if (!(intake.getIntakeSpeed() >= -IntakeConstants.INTAKE_SPEED_DEADBAND
                        && intake.getIntakeSpeed() <= IntakeConstants.INTAKE_SPEED_DEADBAND)) {
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

                // this will make the lights pink if brake mode is enabled
                if (drivetrain.isBrakeMode()) {
                    centerColor = new Color(245, 5, 205);
                }

                Color centerColorWithMultiplier = new Color(centerColor.red * centerMultiplier,
                        centerColor.green * centerMultiplier, centerColor.blue * centerMultiplier);
                for (var i = EDGE_GAP; i < m_ledBuffer.getLength() - EDGE_GAP; i++) {
                    m_ledBuffer.setLED(i, centerColorWithMultiplier);
                }

                setSpeed(drivetrain.getMaxDrivetrainSpeed());
            } else if (inst.isConnected()) {
                // if (timer % 5 == 0) {
                // autoColorMove();
                // }
                autoColorBounce();
                timer++;
            } else {
                centerColor = new Color(0, 255, 0);
                for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setLED(i, centerColor);
                }
            }
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, new Color(255, 0, 0));
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

    private void autoColorMove() {

        // set everything to the alliance color first
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if (DriverStation.getAlliance() == Alliance.Blue) {
                m_ledBuffer.setLED(i, new Color(0, 0, 255));
            } else {
                m_ledBuffer.setLED(i, new Color(205, 0, 0));

            }
        }
        // increase them all by one
        for (int i = 0; i < movingLightsUp.size(); i++) {
            movingLightsUp.set(i, movingLightsUp.get(i) + 1);
        }
        // if the last one is over, subtract the number of leds
        if (movingLightsUp.get(movingLightsUp.size() - 1) >= m_ledBuffer.getLength()) {
            movingLightsUp.set(movingLightsUp.size() - 1,
                    movingLightsUp.get(movingLightsUp.size() - 1) - m_ledBuffer.getLength());
            movingLightsUp.sort(null);
        }

        for (int i = 0; i < movingLightsUp.size(); i++) {

            m_ledBuffer.setLED(movingLightsUp.get(i), new Color(255, 255, 255));

        }

        // START NEW STUFF
        for (int i = 0; i < movingLightsDown.size(); i++) {
            movingLightsDown.set(i, movingLightsDown.get(i) - 1);
        }
        // if the last one is less them one, add the number of leds
        if (movingLightsDown.get(0) < 0) {
            movingLightsDown.set(0, m_ledBuffer.getLength() - 1);
            movingLightsDown.sort(null);
        }

        for (int i = 0; i < movingLightsDown.size(); i++) {

            m_ledBuffer.setLED(movingLightsDown.get(i), new Color(255, 255, 255));

        }
        // END NEW STUFF

    }

    private void autoColorBounce() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if (DriverStation.getAlliance() == Alliance.Blue) {
                m_ledBuffer.setLED(i, new Color(0, 0, 255));
            } else {
                m_ledBuffer.setLED(i, new Color(205, 0, 0));

            }
        }
        if (bounceGoingUp && bouncePosition >= m_ledBuffer.getLength() - BOUNCE_LENGTH) {
            bounceGoingUp = false;
        } else if (!bounceGoingUp && bouncePosition == 0) {
            bounceGoingUp = true;
        }
        if (bounceGoingUp) {
            bouncePosition++;
        } else {
            bouncePosition--;
        }
        for (int i = bouncePosition; i < BOUNCE_LENGTH + bouncePosition; i++) {
            m_ledBuffer.setLED(i, new Color(255, 255, 255));

        }
    }
}
