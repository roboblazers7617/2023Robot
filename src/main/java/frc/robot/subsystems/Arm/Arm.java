// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;


import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pnuematics;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Constants.ArmConstants.WristConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants.ArmPosition;
import frc.robot.Constants.ArmConstants.StateConstants.GenericPosition;
import frc.robot.Constants.ArmConstants.StateConstants.StatePosition;
import frc.robot.Constants.ArmConstants.PnuematicsConstants.PnuematicPositions;
import frc.robot.Constants.ArmConstants.WristConstants.WristPosition;
import frc.robot.subsystems.Arm.States.ArmState;
import frc.robot.subsystems.Arm.States.WaitUntilAtSetpoint;
import frc.robot.subsystems.Arm.States.WaitUntilAtState;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax shoulderMotor = new CANSparkMax(ShoulderConstants.SHOULDER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax shoulderMotorFollower = new CANSparkMax(ShoulderConstants.SHOULDER_FOLLOWER_MOTOR_ID,
      MotorType.kBrushless);
  private SparkMaxPIDController controller;
  private SparkMaxPIDController controllerFollower;
  private RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
  private RelativeEncoder shoulderFollowerEncoder = shoulderMotorFollower.getEncoder();
  private DoubleSolenoid leftPiston;
  private DoubleSolenoid rightPiston;
  private Pnuematics pneumatics;
  private ArmFeedforward feedforward = new ArmFeedforward(ShoulderConstants.KS, ShoulderConstants.KG, ShoulderConstants.KV);
  private final CANSparkMax wristMotor = new CANSparkMax(WristConstants.WRIST_CAN_ID, MotorType.kBrushless);

  private final ArmFeedforward wristFeedforward = new ArmFeedforward(WristConstants.WRIST_KS, WristConstants.WRIST_KG,
      WristConstants.WRIST_KV);

  private final SparkMaxPIDController wristController = wristMotor.getPIDController();

  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
  private Timer time = new Timer();

  private double dt, lastTime;

  private double shoulderSetpoint = ArmPosition.STOW.getShoulderAngle();
  private double wristSetpoint = WristPosition.STOW.angle();

  private double armUpperBound = ShoulderConstants.MAX_SHOULDER_ANGLE;
  private double armLowerBound = ShoulderConstants.MINIMUM_SHOULDER_ANGLE;
  private double wristUpperBound = WristConstants.MAX_WRIST_ANGLE;

  private StatePosition[][] stateDiagram = {
    {null,                           StatePosition.Stow,           StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.ConeLevel3,     StatePosition.CubeLevel3},

    {StatePosition.Stow,             StatePosition.Stow,           StatePosition.LowTransition,   StatePosition.LowTransition,   StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.LowTransition,    StatePosition.Stow,           StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.HighTransition,   StatePosition.Stow,           StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.ConeLevel3,     StatePosition.CubeLevel3},
    {StatePosition.ConeFloorPickup,  StatePosition.LowTransition,  StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.LowTransition,  StatePosition.LowTransition,  StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.CubeFloorPickup,  StatePosition.LowTransition,  StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.LowTransition,  StatePosition.LowTransition,  StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.ConeDoublePickup, StatePosition.Stow,           StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.ConeLevel3,     StatePosition.CubeLevel3},
    {StatePosition.CubeDoublePickup, StatePosition.Stow,           StatePosition.ConeFloorPickup, StatePosition.CubeFloorPickup, StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.ConeLevel3,     StatePosition.CubeLevel3},
    {StatePosition.ConeLevel1,       StatePosition.Stow,           StatePosition.LowTransition,   StatePosition.LowTransition,   StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.CubeLevel1,       StatePosition.Stow,           StatePosition.LowTransition,   StatePosition.LowTransition,   StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.ConeLevel2,       StatePosition.Stow,           StatePosition.LowTransition,   StatePosition.LowTransition,   StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.ConeLevel3,     StatePosition.CubeLevel3},
    {StatePosition.CubeLevel2,       StatePosition.Stow,           StatePosition.LowTransition,   StatePosition.LowTransition,   StatePosition.ConeDoublePickup, StatePosition.CubeDoublePickup, StatePosition.ConeLevel1,     StatePosition.CubeLevel1,     StatePosition.ConeLevel2,     StatePosition.CubeLevel2,     StatePosition.HighTransition, StatePosition.HighTransition},
    {StatePosition.ConeLevel3,       StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition,   StatePosition.ConeDoublePickup, StatePosition.HighTransition,   StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.ConeLevel3,     StatePosition.CubeLevel3},
    {StatePosition.CubeLevel3,       StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition,   StatePosition.ConeDoublePickup, StatePosition.HighTransition,   StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.HighTransition, StatePosition.ConeLevel3,     StatePosition.CubeLevel3}};

    private StatePosition currentState = StatePosition.Stow;
    private StatePosition targetState = StatePosition.Stow;
    private StatePosition interimState = StatePosition.Stow;


  public Arm(Pnuematics pnuematics) {
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotorFollower.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotorFollower.setIdleMode(IdleMode.kBrake);

    shoulderMotor.setSmartCurrentLimit(ShoulderConstants.CURRENT_LIMIT);
    shoulderMotorFollower.setSmartCurrentLimit(ShoulderConstants.CURRENT_LIMIT);

    shoulderMotorFollower.follow(shoulderMotor, true);

    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);
    wristMotor.setIdleMode(IdleMode.kBrake);

    shoulderEncoder.setPositionConversionFactor(ShoulderConstants.POSITION_CONVERSION_FACTOR);
    shoulderEncoder.setVelocityConversionFactor(ShoulderConstants.POSITION_CONVERSION_FACTOR / 60.0);
    shoulderEncoder.setPosition(ShoulderConstants.MINIMUM_SHOULDER_ANGLE);
    shoulderFollowerEncoder.setPositionConversionFactor(ShoulderConstants.POSITION_CONVERSION_FACTOR);
    shoulderFollowerEncoder.setVelocityConversionFactor(ShoulderConstants.POSITION_CONVERSION_FACTOR / 60.0);
    shoulderFollowerEncoder.setPosition(ShoulderConstants.MINIMUM_SHOULDER_ANGLE);

    wristEncoder.setPositionConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR);
    wristEncoder.setVelocityConversionFactor(WristConstants.WRIST_ENCODER_CONVERSION_FACTOR / 60);
    wristEncoder.setPosition(WristConstants.MAX_WRIST_ANGLE);

    controller = shoulderMotor.getPIDController();
    controllerFollower = shoulderMotorFollower.getPIDController();

    controllerFollower.setP(ShoulderConstants.KP);
    controllerFollower.setI(ShoulderConstants.KI);
    controllerFollower.setD(ShoulderConstants.KD);
    controllerFollower.setOutputRange(ShoulderConstants.MAX_SPEED_DOWNWARD, ShoulderConstants.MAX_SPEED_UPWARD);
    controllerFollower.setSmartMotionMaxAccel(ShoulderConstants.MAX_ACCEL, 0);
    controllerFollower.setSmartMotionMaxVelocity(ShoulderConstants.MAX_VEL, 0);

    controller.setP(ShoulderConstants.KP);
    controller.setI(ShoulderConstants.KI);
    controller.setD(ShoulderConstants.KD);
    controller.setOutputRange(ShoulderConstants.MAX_SPEED_DOWNWARD, ShoulderConstants.MAX_SPEED_UPWARD);
    controller.setSmartMotionMaxAccel(ShoulderConstants.MAX_ACCEL, 0);
    controller.setSmartMotionMaxVelocity(ShoulderConstants.MAX_VEL, 0);

    controller.setP(ShoulderConstants.KP);

    wristController.setP(WristConstants.WRIST_KP);
    wristController.setI(WristConstants.WRIST_KI);
    wristController.setD(WristConstants.WRIST_KD);
    wristController.setOutputRange(WristConstants.MAX_DOWNWARD_WRIST_SPEED, WristConstants.MAX_UPWARD_WRIST_SPEED);
    wristController.setSmartMotionMaxAccel(WristConstants.MAX_ACCEL, 0);
    wristController.setSmartMotionMaxVelocity(WristConstants.MAX_VEL, 0);
    wristController.setReference(wristSetpoint, CANSparkMax.ControlType.kSmartMotion, 0,
    wristFeedforward.calculate(Units.degreesToRadians(wristSetpoint), 0));

    this.pneumatics = pnuematics;
    leftPiston = pnuematics.getLeftArmPiston();
    rightPiston = pnuematics.getRightArmPiston();

    time.reset();
    time.start();

    actuateSuperstructure(PnuematicPositions.RETRACTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dt = time.get() - lastTime;
    lastTime = time.get();

  }

  public void turnOnBrakes(Boolean isBraked) {
    if (isBraked) {
      shoulderMotor.setIdleMode(IdleMode.kBrake);
      shoulderMotorFollower.setIdleMode(IdleMode.kBrake);
      wristMotor.setIdleMode(IdleMode.kBrake);
    } else {
      shoulderMotor.setIdleMode(IdleMode.kCoast);
      shoulderMotorFollower.setIdleMode(IdleMode.kCoast);
      wristMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  public void setPosition(double shoulderPosition, double wristPosition) {
    shoulderSetpoint = Math.min(shoulderPosition, armUpperBound);
    shoulderSetpoint = Math.max(shoulderSetpoint, armLowerBound);
    wristSetpoint = Math.min(wristUpperBound, wristPosition);
    controller.setReference(shoulderSetpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(shoulderSetpoint), 0));
    controllerFollower.setReference(shoulderSetpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(shoulderSetpoint), 0));
    wristController.setReference(wristSetpoint, CANSparkMax.ControlType.kPosition, 0,
     wristFeedforward.calculate(Units.degreesToRadians(wristSetpoint + (getArmAngle()-ShoulderConstants.MINIMUM_SHOULDER_ANGLE)), 0));
  }

  private void setPosition(double shoulderPosition, double wristPosition, PnuematicPositions pnuematicPosition) {
    setPosition(shoulderPosition, wristPosition);
    actuateSuperstructure(pnuematicPosition);
  }


  private void setPosition(ArmPosition shoulderPosition, WristPosition wristPosition) {
    setPosition(shoulderPosition.getShoulderAngle(), wristPosition.angle(), shoulderPosition.getPistonPosition());

  }
  /** Please Please Please try to only use this in subpackages, it really should not be exposed elsewhere. */
  public void setPosition(StatePosition state){
    setPosition(state.getArmPosition(), state.getWristPosition());
    currentState = state;
  }

  public void setShoulderVelocity(double velocityDegreesPerSec) {
    shoulderSetpoint += (velocityDegreesPerSec * dt);
    shoulderSetpoint = Math.min(shoulderSetpoint, armUpperBound);
    shoulderSetpoint = Math.max(shoulderSetpoint, armLowerBound);
    controller.setReference(shoulderSetpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(shoulderSetpoint), Units.degreesToRadians(velocityDegreesPerSec)));
    controllerFollower.setReference(shoulderSetpoint, CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(Units.degreesToRadians(shoulderSetpoint), Units.degreesToRadians(velocityDegreesPerSec)));
  }

  public void setWristVelocity(double velocityDegreesPerSec){
    wristSetpoint += (velocityDegreesPerSec * dt);  
    wristSetpoint = Math.min(velocityDegreesPerSec, wristUpperBound);
    wristController.setReference(wristSetpoint, CANSparkMax.ControlType.kPosition, 0,
     wristFeedforward.calculate(Units.degreesToRadians(wristSetpoint + (getArmAngle()-ShoulderConstants.MINIMUM_SHOULDER_ANGLE)), velocityDegreesPerSec));
  }

  public void setJointVelocities(Supplier<Double> shoulderVelocity, Supplier<Double> wristVelocity){
    setShoulderVelocity(shoulderVelocity.get());
    setWristVelocity(wristVelocity.get());
  }

  

  public void actuateSuperstructure(PnuematicPositions positions) {
    leftPiston.set(positions.getValue());
    rightPiston.set(positions.getValue());
  }

  public void resetEncoders(){
    shoulderEncoder.setPosition(ShoulderConstants.MINIMUM_SHOULDER_ANGLE);
    shoulderFollowerEncoder.setPosition(ShoulderConstants.MINIMUM_SHOULDER_ANGLE);
    wristEncoder.setPosition(WristConstants.MAX_WRIST_ANGLE);
    setPosition(ShoulderConstants.MINIMUM_SHOULDER_ANGLE, WristConstants.MAX_WRIST_ANGLE);
  }

  public void enableCompressor(boolean enableCompressor) {
    if (enableCompressor) {
      pneumatics.enable();
    } else {
      pneumatics.disable();
    }
  }

  public void removeBounds(){
    armUpperBound = Integer.MAX_VALUE;
    armLowerBound = Integer.MIN_VALUE;
    wristUpperBound = Integer.MAX_VALUE;
  }

  public void addBounds(){
    armUpperBound = ShoulderConstants.MAX_SHOULDER_ANGLE;
    armLowerBound = ShoulderConstants.MINIMUM_SHOULDER_ANGLE;
    wristUpperBound = WristConstants.MAX_WRIST_ANGLE;
  }
//GETTERS  | | |
//         U U U



  public double getShoulderAngle() {
    return shoulderEncoder.getPosition();
  }

  public double getWristAngle() {
    return wristEncoder.getPosition();
  }

  public Value getSuperstructureState() {
    return leftPiston.get();
  }

  public double getArmAngle() {
    return shoulderEncoder.getPosition()
        + ((getSuperstructureState() == Value.kForward) ? ShoulderConstants.PISTON_BACK : ShoulderConstants.PISTON_FORWARD);
  }

  public double getWristVelocity(){
    return wristEncoder.getVelocity();
  }

  public boolean shoulderAtSetpoint() {
    return (Math.abs(getShoulderAngle() - (shoulderSetpoint)) < (ShoulderConstants.POSITION_TOLERANCE));
  }

  public boolean wristAtSetpoint(){
    return (Math.abs(getWristAngle() - wristSetpoint) < (WristConstants.WRIST_ANGLE_TOLERANCE));
  }

  public boolean atSetpoints(){
    return shoulderAtSetpoint() && wristAtSetpoint() && (getSuperstructureState() == currentState.getArmPosition().getPistonPosition().getValue());
  }


  public double getShoulderMotorTemp() {
    return shoulderMotor.getMotorTemperature();
  }

  public double getWristMotorTemp(){
    return wristMotor.getMotorTemperature();
  }
//STATE MACHINE STUFF BELOW | | |
//                          U U U



  private StatePosition evalState(PieceType piece, GenericPosition position) throws Exception{
    if(position == GenericPosition.Stow){
      return StatePosition.Stow;
    }
    for(int i = 1; i<stateDiagram.length; i++){
      if((stateDiagram[i][0].name().contains(piece.name())) && (stateDiagram[i][0].name().contains(position.name())))
              return stateDiagram[i][0];
    }
   throw new Exception("The state you are trying to reach does not exist");
  }

  private StatePosition iterateOverStateDiagram() throws Exception{
    if(currentState != targetState){
      int targetIndex = 0;
      int currentIndex = 0;
      for(int i = 1; i<stateDiagram[0].length; i++){
          if(stateDiagram[0][i] == targetState)
              targetIndex = i;
      }
      for(int j = 1; j<stateDiagram.length; j++){
              if(stateDiagram[j][0] == targetState)
                      currentIndex = j;
      }
      if((targetIndex != 0) && (currentIndex != 0))
          return stateDiagram[currentIndex][targetIndex];
          else
            throw new Exception("State diagram failed to process");
  
      }
      return currentState;
  }

  public Command changeState(PieceType piece, GenericPosition position, boolean waitUntilAtSetpoint){
    SequentialCommandGroup commandToReturn = new SequentialCommandGroup(Commands.none());
    try{
      targetState = evalState(piece, position);
    }
    catch(Exception e){
      System.out.println(e.getMessage());
      return Commands.none();
    }
    try{
      interimState = iterateOverStateDiagram();
    }
    catch(Exception e){
      System.out.println(e.getMessage());
      return Commands.none();
    }
    commandToReturn.addCommands(new ArmState(this, interimState));

    while(interimState != targetState){
      try{
        interimState = iterateOverStateDiagram();
      }
      catch(Exception e){
        System.out.println(e.getMessage());
        return Commands.none();
      }
      commandToReturn.addCommands(new WaitUntilAtSetpoint(this),(new ArmState(this, interimState)));
    }
    if(waitUntilAtSetpoint)
        commandToReturn.addCommands(new WaitUntilAtState(this));
    return commandToReturn;
    }

    public Command changeState(Supplier<PieceType> piece, GenericPosition position, boolean waitUntilAtSetpoint){
      return changeState(piece.get(), position, waitUntilAtSetpoint);
    }

    public Command changeState(Supplier<PieceType> piece, Supplier<GenericPosition> position, boolean waitUntilAtSetpoint){
      return changeState(piece.get(), position.get(), waitUntilAtSetpoint);
    }

    public StatePosition getCurrentState() {
        return currentState;
    }

    public StatePosition getTargetState() {
        return targetState;
    }

  }
