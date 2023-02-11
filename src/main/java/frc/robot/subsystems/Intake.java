// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final double maxIntakeRotation = 270.0;
  private final double WRIST_ANGULAR_TOLERANCE = 0.5;
  private final double WRIST_MAX_VELOCITY = 0.26;
  private final double WRIST_MAX_ACCEL = 0.13;
  private final int CURRENT_LIMIT = 40;
  private final int WRIST_LIMIT_SWITCH_CHANNEL = 2;
  private final int WRIST_POT_CHANNEL = 1;
  private final double WRIST_POT_SCALE = 4.71;
  private final int WRIST_CAN_ID = 74;
  private final int INTAKE_CAN_ID = 75;
  private final double KP_ANGULAR = 3.84;
  private final double KI_ANGULAR = 0.0;
  private final double KD_ANGULAR = 0.0;


  private final CANSparkMax wristMotor = new CANSparkMax(WRIST_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_CAN_ID, MotorType.kBrushless);

  private final ProfiledPIDController wristPID = new ProfiledPIDController(
            KP_ANGULAR, KI_ANGULAR, KD_ANGULAR, new TrapezoidProfile.Constraints(WRIST_MAX_VELOCITY, WRIST_MAX_ACCEL));

  private final AnalogPotentiometer wristPot = new AnalogPotentiometer(WRIST_POT_CHANNEL, WRIST_POT_SCALE);

  private final DigitalInput isIntakeStored = new DigitalInput(WRIST_LIMIT_SWITCH_CHANNEL);


  // All angles in radians
  public  enum WristPosition
  {
    STORED (0.0),
    THIRD_LEVEL (0.26),
    PICKUP_STATION (.26),
    SECOND_LEVEL (0.52),
    GROUND (1.04);

    private final double angleRadians;
    WristPosition(double angle){
      this.angleRadians = angle;
    }
    public double angle() {return angleRadians; }
  }

  public Intake() {
    wristMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    wristMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    wristMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    wristPID.setTolerance(WRIST_ANGULAR_TOLERANCE);

  }

  @Override
  public void periodic() {
  }

  // Assumes wrist movement to store is a positive speed and that wrist begins at zero degrees (stored)
  public void setWristSpeed(double speed)
  {
    if (isIntakeAtMaxRotataion() )
    if ((speed > 0.0) && !isIntakeAtMaxRotataion()){
      wristMotor.set(MathUtil.clamp(speed, -WRIST_MAX_VELOCITY, WRIST_MAX_VELOCITY));
    }
    else if ((speed < 0.0) && !isIntakeStored())
    {
      wristMotor.set(MathUtil.clamp(speed, -WRIST_MAX_VELOCITY, WRIST_MAX_VELOCITY));
    }
    else 
    {
      wristMotor.set(0.0);
    }      
  }

  public Command MoveWristCommand(WristPosition targetPosition)
  {
    return new FunctionalCommand(
                  () -> this.wristPID.setGoal(targetPosition.angle()), 
                  () -> this.setWristSpeed(wristPID.calculate(this.getWristAngle())),
                  onEnd -> this.setWristSpeed(0.0), 
                  () -> this.wristPID.atGoal(), 
                  this);
  }

  public Command Move(WristPosition targetPosition)
  {
    return new ProfiledPIDCommand(
      new ProfiledPIDController(
            // The PID gains
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)), 
      () -> this.getWristAngle(), 
      targetPosition.angle(), 
      (output, setpoint) -> { 
        // Use the output (and setpoint, if desired) here
        ArmFeedforward m_feedforward =
        new ArmFeedforward(
            0, 0,
            0, 0);
        this.setWristSpeed(output + m_feedforward.calculate(setpoint.position, setpoint.velocity));
      }, 
      this);
  }

  public boolean isIntakeStored()
  {
    return isIntakeStored.get();
  }
  public boolean isIntakeAtMaxRotataion(){
    return (wristPot.get() >= maxIntakeRotation);
  }
  public double getWristAngle()
  {
    return wristPot.get();
  }


}
