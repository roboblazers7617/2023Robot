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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final Vision mVision;

  private final DifferentialDrive drivetrain;
  private DrivetrainConstants.DrivetrainMode mode;
  private double maxDrivetrainspeed = DrivetrainConstants.REG_SPEED;

  private Translation2d targetTranslation;

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
    mOdometry = new DifferentialDrivePoseEstimator(mKinematics, getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d());

  }

  @Override
  public void periodic() {
    updatePose();
  }

  public void drive(double leftY, double rightX, double rightY) {
    double lForward = slewRateFilterLeft.calculate(leftY);
    double rForward = slewRateFilterRight.calculate(rightY);
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

    motorController.setIdleMode(IdleMode.kCoast);
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
    return mGyro.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public DifferentialDriveKinematics getKinematics(){
    return mKinematics;
  }

  public void resetOdometry(Pose2d pose){
    mOdometry.resetPosition(getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    leftFrontMotor.setVoltage(leftVolts * 12); // Convert this from percent of battery to volts by multiply by 12
    rightFrontMotor.setVoltage(rightVolts * 12); // Convert this from percent of battery to volts by multiply by 12
  }

  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }

  public void zeroHeading(){
    mGyro.reset();
  }
  public double getAngle(){
    return -mGyro.getAngle();
  }

  private void updatePose() {
    // Write code for local Odometry here:
    mOdometry.update(mGyro.getRotation2d(), getLeftDistance(), getRightDistance());

    Optional<EstimatedRobotPose> cameraPose = mVision.getEstimatedGlobalPose(getPose2d());
    if(cameraPose.isPresent()){
    mOdometry.addVisionMeasurement(cameraPose.get().estimatedPose.toPose2d(), cameraPose.get().timestampSeconds);
    mGyro.setYaw(cameraPose.get().estimatedPose.getRotation().getAngle());
    }
  }
  public Command PickPathWork(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle) {
    PathPlannerTrajectory test_path = PathPlanner.generatePath(
      new PathConstraints(.1, .1),
      new PathPoint(new Translation2d(x.getAsDouble(),y.getAsDouble()), new Rotation2d(angle.getAsDouble())),
     //new PathPoint(new Translation2d(12.75, 1), Rotation2d.fromDegrees(180)),
      new PathPoint(new Translation2d(14.61, 1.07), Rotation2d.fromDegrees(0)));
  
    SmartDashboard.putNumber("x", x.getAsDouble());
    SmartDashboard.putNumber("y", y.getAsDouble());
    SmartDashboard.putNumber("angle", angle.getAsDouble());
  
    PPRamseteCommand returnCommand = new PPRamseteCommand(
        test_path, 
        drivetrain::getPose2d, 
        new RamseteController(), 
        new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV),
        drivetrain.getKinematics(),
        drivetrain::getWheelSpeeds,
        new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
        new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
        drivetrain::tankDriveVolts,
        false,
        drivetrain);
    return returnCommand;
    }
    public Translation2d getTargetTranslation() {
      return new Translation2d(1,1);
    }

    public double getaverageEncoderDistance() {
      return (getLeftDistance() + getRightDistance()) / 2;
    }
  }
  
