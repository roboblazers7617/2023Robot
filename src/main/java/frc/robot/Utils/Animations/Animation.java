// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils.Animations;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public abstract class Animation {
    private Timer time = new Timer();
    public double minUpdatePeriod = 0.01;
    private int cyclesRan = 0;
    protected HashMap<Integer, Color> generatedHashmap;
    
    public Animation(){
        time.start();
    }

    public Animation(int minUpdatePeriod){
        time.start();
        this.minUpdatePeriod = minUpdatePeriod;
    }

    public void update(){
        if(time.advanceIfElapsed(minUpdatePeriod)){
            cyclesRan++;
            generatePattern();
        }

    }

    public abstract HashMap<Integer, Color> generatePattern();

    public void reset(){
        cyclesRan = 0;
    }

    public int getCyclesRan(){
        return cyclesRan;
    }

    public void replaceElementKey(int key, int newKey){
        Color replacedColor = generatedHashmap.get(key);
        generatedHashmap.remove(key);
        generatedHashmap.put(newKey, replacedColor);
    }

}
