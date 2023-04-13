// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils.Animations;

import java.util.HashMap;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class RainbowFadeAnimation extends Animation {
    private int hue;
    private int startingHue;
    private int saturation;
    private int value;
    public RainbowFadeAnimation(int LEDStripLength){
        super(LEDStripLength);
        hue = 60;
        saturation = 255;
        value = 128;
    }

    public RainbowFadeAnimation(int LEDStripLength, int hue){
        super(LEDStripLength);
        this.hue = hue;
        startingHue = hue;
        saturation = 255;
        value = 128;
    }

    public RainbowFadeAnimation(int LEDStripLength, int hue, int saturation, int value){
        super(LEDStripLength);
        this.hue = hue = startingHue;
        this.saturation = saturation;
        this.value = value;
    }


    @Override
    public void reset(){
        super.reset();
        hue = startingHue;
    }

    @Override
    public HashMap<Integer, Color> generatePattern() {
        hue += 1;
        //180 is the max value for a hue in HSV
        if(hue == 180)
            hue = 0;
        generatedHashmap.put(LEDStripLength, Color.fromHSV(hue, saturation, value));
        return generatedHashmap;
    }

    

}
