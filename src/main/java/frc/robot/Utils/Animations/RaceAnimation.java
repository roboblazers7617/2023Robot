// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils.Animations;

import java.util.HashMap;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class RaceAnimation extends Animation {
    private final Color backgroundColor, raceColor;
    private Color previousStripColor;
    private final int raceLength, raceGap, LEDStripLength;

    public RaceAnimation(Color backgroundColor, Color raceColor, int raceLength, int raceGap, int LEDStripLength) {
        super();
        this.backgroundColor = backgroundColor;
        this.raceColor = raceColor;
        this.raceLength = raceLength;
        this.raceGap = raceGap;
        this.LEDStripLength = LEDStripLength;
        previousStripColor = backgroundColor;
    }

    @Override
    public void reset() {
        super.reset();
        generatedHashmap = null;
        previousStripColor = backgroundColor;
    }

    @Override
    public HashMap<Integer, Color> generatePattern() {
        if (generatedHashmap == null) {
            generatedHashmap.put(raceLength, raceColor);
            generatedHashmap.put(LEDStripLength, backgroundColor);
        } else {
            // Increment existing strips
            for (int stripEnd : generatedHashmap.keySet()) {
                replaceElementKey(stripEnd, stripEnd + 1);
            }
            for (int stripEnd : generatedHashmap.keySet()) {
                // Need to Generate a new background strip
                if ((stripEnd == raceLength + 1) && (generatedHashmap.get(stripEnd) == raceColor))
                    generatedHashmap.put(1, backgroundColor);

                // Need to generate new race strip
                else if ((stripEnd == raceGap + 1) && (generatedHashmap.get(stripEnd) == backgroundColor))
                    generatedHashmap.put(1, raceColor);
                
                //Remove strips that are not visable
                else if(stripEnd == LEDStripLength + raceLength && (generatedHashmap.get(stripEnd) == raceColor))
                    generatedHashmap.remove(stripEnd);
                else if(stripEnd == LEDStripLength + raceGap && (generatedHashmap.get(stripEnd) == backgroundColor))
                    generatedHashmap.remove(stripEnd);
            }
        }
        return generatedHashmap;
    }

}
