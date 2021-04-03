// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {

  public static final int backLeftDriveMotor = 1;
    public static final int backRightDriveMotor = 2;
    public static final int frontLeftDriveMotor = 3;
    public static final int frontRightDriveMotor = 4;
  
    public final static WPI_TalonFX frontLeft = new WPI_TalonFX(frontLeftDriveMotor);
    public final WPI_TalonFX backLeft = new WPI_TalonFX(backLeftDriveMotor);
    public final static WPI_TalonFX frontRight = new WPI_TalonFX(frontRightDriveMotor);
    public final WPI_TalonFX backRight = new WPI_TalonFX(backRightDriveMotor);

    // The motors on the left side of the drive.
    private final SpeedControllerGroup left = new SpeedControllerGroup(frontLeft);//, backLeft);

    // Define right Speed Controller
    private final SpeedControllerGroup right = new SpeedControllerGroup(frontRight);//, backRight);

    private final DifferentialDrive m_drive = new DifferentialDrive(left, right);

    // The gyro sensor
    private final Gyro m_gyro = new AHRS();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
      frontLeft.configFactoryDefault();
      backLeft.configFactoryDefault();
      frontRight.configFactoryDefault();
      backRight.configFactoryDefault();

      backLeft.follow(frontLeft);
      backRight.follow(frontRight);

      frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      frontLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
      backLeft.setNeutralMode(NeutralMode.Brake);
      backRight.setNeutralMode(NeutralMode.Brake);

      // Sets the distance per pulse for the encoders
      // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block

      m_odometry.update(m_gyro.getRotation2d(),
          frontLeft.getSelectedSensorPosition() / (Constants.ENCODER_EDGES_PER_REV * Constants.gearRatio
              * Units.inchesToMeters(Constants.wheelCircumferenceInches)),
          frontRight.getSelectedSensorPosition() / (Constants.ENCODER_EDGES_PER_REV * Constants.gearRatio
              * Units.inchesToMeters(Constants.wheelCircumferenceInches)));
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(
          (frontLeft.getSelectedSensorPosition() * 10) / (Constants.ENCODER_EDGES_PER_REV * Constants.gearRatio
              * Units.inchesToMeters(Constants.wheelCircumferenceInches)),
          (frontRight.getSelectedSensorVelocity() * 10) / (Constants.ENCODER_EDGES_PER_REV * Constants.gearRatio
              * Units.inchesToMeters(Constants.wheelCircumferenceInches)));
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      // m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      left.setVoltage(leftVolts);
      right.setVoltage(rightVolts);
      m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public static void resetEncoders() {
      frontLeft.setSelectedSensorPosition(0);
      frontRight.setSelectedSensorPosition(0);
}

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftDistance() {
    return (frontLeft.getSelectedSensorPosition(0));
}

public double getRightDistance() {
    return (frontRight.getSelectedSensorPosition(0) * -1);// * 0.001;
}

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
