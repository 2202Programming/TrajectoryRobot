package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class DriveSubsystem extends SubsystemBase {
  //control the inverstion as best we can
  // changing conversion factors on SparkMax don't seem to take signs
  // setInverted doesn't work when using Neo brushless motors
  final double KleftSign = -1.0;     // adjust left/right so positive is going 
  final double KrightSign = 1.0;     // forward on the robot.  
  final double Kgyro = -1.0;         // ccw is positive, just like geometry class

  // just for notes
  private final CANSparkMaxLowLevel.MotorType MT = CANSparkMaxLowLevel.MotorType.kBrushless;
  private final CANSparkMax frontRight = new CANSparkMax(CAN.FR_SMAX, MT);
  private final CANSparkMax frontLeft = new CANSparkMax(CAN.FL_SMAX, MT);
  private final CANSparkMax backRight = new CANSparkMax(CAN.BR_SMAX, MT);
  private final CANSparkMax backLeft = new CANSparkMax(CAN.BL_SMAX, MT);
  private final CANSparkMax middleRight = new CANSparkMax(CAN.MR_SMAX, MT);
  private final CANSparkMax middleLeft = new CANSparkMax(CAN.ML_SMAX, MT);
  private final CANEncoder left_encoder = backLeft.getEncoder();
  private final CANEncoder right_encoder = backRight.getEncoder();

  // feet per motor rotation = wheel circumference / gearbox ratio
  private final double kFeetPerRotation = (Math.PI * Constants.WHEEL_DIAMETER) / Constants.LOW_GEAR_RATIO;

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(backRight);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(backLeft);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  //tracking, updated in periodic, scaled and sign corrected physical units, robot frame
  double m_velLeft;   
  double m_velRight;
  double m_posLeft;
  double m_posRight;
  double m_theta;    //heading   
  double m_voltRight;
  double m_voltLeft;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // backRight.setInverted(true); //invert right side motors
    //left_encoder.setPositionConversionFactor(-1.0);

    middleRight.follow(backRight);
    frontRight.follow(backRight);
    middleLeft.follow(backLeft);
    frontLeft.follow(backLeft);

    // this should cause a 1-2 second delay
    m_gyro.calibrate();
    while (m_gyro.isCalibrating()) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(250);
        System.out.println("calibrating gyro");
      } catch (InterruptedException e) {
  
      }
    }
    System.out.println("Pre-Zero yaw:" + m_gyro.getYaw());

    m_gyro.reset(); //should zero yaw but not working.
    m_gyro.zeroYaw(); //should zero yaw but not working.

    System.out.println("Post-Zero yaw:" + m_gyro.getYaw());

    resetEncoders();
    m_drive.setSafetyEnabled(false);  //for debugging
    m_odometry = new DifferentialDriveOdometry(readGyro());
  }

  @Override
  public void periodic() {
    //read and scale everything at once
    m_velLeft = left_encoder.getVelocity()*kFeetPerRotation*KleftSign;   
    m_posLeft = left_encoder.getPosition()*kFeetPerRotation*KleftSign;
    m_velRight = right_encoder.getVelocity()*kFeetPerRotation*KleftSign;
    m_posRight = right_encoder.getPosition()*kFeetPerRotation*KrightSign;
    m_theta = Kgyro*m_gyro.getYaw();

    // Update the odometry in the periodic block, physical units
    m_odometry.update(readGyro(), m_posLeft, m_posRight);

    // share for debugging
    SmartDashboard.putNumber("DT/Heading", m_theta);
    SmartDashboard.putNumber("DT/HeadingDot", getTurnRate());
    SmartDashboard.putNumber("DT/L_Odometer", m_posLeft);
    SmartDashboard.putNumber("DT/R_Odometer", m_posRight);
    SmartDashboard.putNumber("DT/L_Vel", m_velLeft);
    SmartDashboard.putNumber("DT/R_Vel", m_velRight);
    SmartDashboard.putNumber("DT/L_Volts", m_voltLeft);
    SmartDashboard.putNumber("DT/R_Volts", m_voltRight);
  }

  //Need to use getYaw to get -180 to 180 as expected.
  Rotation2d readGyro() {
    return Rotation2d.fromDegrees(m_theta); 
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
    return new DifferentialDriveWheelSpeeds(m_velLeft, m_velRight);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());  // has a -1 in the interface for they gyro
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_voltLeft = KleftSign * leftVolts;
    m_voltRight = KrightSign * rightVolts;
    m_leftMotors.setVoltage(m_voltLeft);
    m_rightMotors.setVoltage(m_voltRight);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders. Physical units.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_posLeft + m_posRight) / 2.0;
  }

  /*  DPL hid encoder access, not sure why that would be public */
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  /*public*/ CANEncoder getLeftEncoder() {
    return left_encoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  /*public*/ CANEncoder getRightEncoder() {
    return right_encoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return readGyro().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double heading = Kgyro*m_gyro.getRate();
    
    return heading;
  }
}

