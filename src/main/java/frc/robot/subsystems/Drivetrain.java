// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DrivetrainConstants.LEFT_WHEEL_PORT, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DrivetrainConstants.RIGHT_WHEEL_PORT,
      MotorType.kBrushless);
  private final CANSparkMax leftFollowerMotor = new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_WHEEL_PORT,
      MotorType.kBrushless);
  private final CANSparkMax rightFollowerMotor = new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_WHEEL_PORT,
      MotorType.kBrushless);

  private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftFollowerMotor);
  private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightFollowerMotor);

  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder leftFollowerEncoder = leftFollowerMotor.getEncoder();
  private final RelativeEncoder rightFollowerEncoder = rightFollowerMotor.getEncoder();

  private final WPI_Pigeon2 mGyro = new WPI_Pigeon2(DrivetrainConstants.GYRO_ID);  
  private final DifferentialDrivePoseEstimator mOdometry;
  private final DifferentialDriveKinematics mKinematics;
  
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DrivetrainConstants.KS,
      DrivetrainConstants.KV, DrivetrainConstants.KA);
  
      private final Vision mVision;

  private final DifferentialDrive drivetrain;
  private DrivetrainConstants.DrivetrainMode mode;
  private double maxDrivetrainspeed = DrivetrainConstants.REG_SPEED;

  private Pose2d targetPose;

  private SlewRateLimiter slewRateFilterLeft = new SlewRateLimiter(1.0/ DrivetrainConstants.RAMP_TIME_SECONDS);
  private SlewRateLimiter slewRateFilterRight = new SlewRateLimiter(1.0/ DrivetrainConstants.RAMP_TIME_SECONDS);
  public void setDriveTrainMode(DrivetrainConstants.DrivetrainMode mode) {
    this.mode = mode;
  }

  public Drivetrain(Vision vision) {
    drivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    drivetrain.setMaxOutput(DrivetrainConstants.REG_SPEED);
    mode = DrivetrainConstants.DrivetrainMode.tankDrive;

    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftFollowerMotor.restoreFactoryDefaults();
    rightFollowerMotor.restoreFactoryDefaults();

    configureEncoder(leftFrontEncoder);
    configureEncoder(rightFrontEncoder);
    configureEncoder(leftFollowerEncoder);
    configureEncoder(rightFollowerEncoder);

    configureMotor(leftFrontMotor);
    configureMotor(rightFrontMotor);
    configureMotor(leftFollowerMotor);
    configureMotor(rightFollowerMotor);

    rightFrontMotor.setInverted(true);
    rightFollowerMotor.setInverted(true);
    drivetrain.setDeadband(.1);

    mVision = vision;
    mGyro.reset();
    mKinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH_METERS);
    mOdometry = new DifferentialDrivePoseEstimator(mKinematics, mGyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d());

  }

  @Override
  public void periodic() {
    updatePose();
  }

  public void drive(double leftY, double rightX, double rightY) {
    double lForward = //slewRateFilterLeft.calculate
    (leftY);
    double rForward = //slewRateFilterRight.calculate
    (rightY);
    if (mode == DrivetrainConstants.DrivetrainMode.arcadeDrive) {
      arcadeDrive(-lForward, -rightX);
    } else if (mode == DrivetrainConstants.DrivetrainMode.tankDrive) {
      tankDrive(-lForward, -rForward);
    }
  }

  private void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  public void setDrivetrainSpeed(double maxSpeed) {
    maxDrivetrainspeed = maxSpeed;
    drivetrain.setMaxOutput(maxSpeed);
  }

  public double getCarmax() {
    return maxDrivetrainspeed;
  }

  private void configureMotor(CANSparkMax motorController) {

    motorController.setIdleMode(IdleMode.kBrake);
    motorController.setSmartCurrentLimit(DrivetrainConstants.CURRENT_LIMIT);
  }

  private void configureEncoder(RelativeEncoder motorEncoder) {
    motorEncoder.setPositionConversionFactor(DrivetrainConstants.DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION);
    motorEncoder.setPosition(0);
    motorEncoder.setVelocityConversionFactor(DrivetrainConstants.DRIVETRAIN_ENCODER_VELOCITY);
  }

  public double getLeftVelocity() {
    return leftFrontEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightFrontEncoder.getVelocity();
  }

  public double getLeftDistance() {
    return leftFrontEncoder.getPosition();
  }

  public double getRightDistance() {
    return rightFrontEncoder.getPosition();
  }

  public Pose2d getPose2d(){
    return mOdometry.getEstimatedPosition();
  }

  public Rotation2d getRotation2d(){
    return mOdometry.getEstimatedPosition().getRotation();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public DifferentialDriveKinematics getKinematics(){
    return mKinematics;
  }

  public void resetOdometry(Pose2d pose){
    mOdometry.resetPosition(mGyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
  }

  public void driveWithVelocity(double xVelocity, double rotationVelocity){
    var wheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xVelocity, 0.0, rotationVelocity));
    setSpeeds(wheelSpeeds);

  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    var leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);
    leftFrontMotor.setVoltage(leftFeedforward);
    rightFrontMotor.setVoltage(rightFeedforward);
    drivetrain.feed();
  }

  public void setBrakeMode(IdleMode mode)
  {
    leftFrontMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftFollowerMotor.setIdleMode(mode);
    rightFollowerMotor.setIdleMode(mode);

  }
  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }

  public void zeroHeading(){
    mGyro.setYaw(0);
  }
  public double getGyroAngle(){
    return -mGyro.getAngle();
  }

  private void updatePose() {
    // Write code for local Odometry here:
    mOdometry.update(mGyro.getRotation2d(), getLeftDistance(), getRightDistance());

    Optional<EstimatedRobotPose> cameraPose = mVision.getEstimatedGlobalPose(getPose2d().plus(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI))));
    if(cameraPose.isPresent()){
    mOdometry.addVisionMeasurement(cameraPose.get().estimatedPose.toPose2d(), cameraPose.get().timestampSeconds);
    //mGyro.setYaw(mOdometry.getEstimatedPosition().getRotation().getDegrees());
    }
  }
    public Translation2d getTargetTranslation() {
      return new Translation2d(1,1);
    }

    public double getaverageEncoderDistance() {
      return (getLeftDistance() + getRightDistance()) / 2;
    }

    public Command ResetRobotToStartPosition()
    {
      return Commands.runOnce(

       () -> { 
          resetEncoders();
          zeroHeading();
          resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
      }); 
    }
  }
  
