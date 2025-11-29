package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core.components.SmartNavX;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonSRX backLeft;
    private WPI_VictorSPX backRight;
    private WPI_TalonSRX frontLeft;
    private WPI_TalonSRX frontRight;
    private DifferentialDrive drive;
    private final DifferentialDriveOdometry odometry;
    private Encoder encoderRight;
    private Encoder encoderLeft;
    private final SmartNavX navX;

    public Drivetrain() {
        backLeft = new WPI_TalonSRX(4);
        backRight = new WPI_VictorSPX(1);
        frontLeft = new WPI_TalonSRX(3);
        frontRight = new WPI_TalonSRX(2);

        backLeft.setInverted(true);
        backRight.setInverted(false);
        frontLeft.setInverted(true);
        frontRight.setInverted(false);
        this.navX = new SmartNavX();
        backLeft.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);
        frontLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
        drive = new DifferentialDrive(frontLeft, frontRight);

        this.odometry = new DifferentialDriveOdometry(
            this.getRotation2d(),
            0,
            0);

        this.encoderLeft = new Encoder(
                9,
                8,
                false);

        this.encoderRight = new Encoder(
                7,
                6,
                false);
    }
    @Override
    public void periodic() {
      this.debugSmartDashboard(true);
      this.updateOdometry();
    }

    public void resetEncoders() {
        this.encoderLeft.reset();
        this.encoderRight.reset();
    }

    public Encoder getEncoderLeft() {
        return this.encoderLeft;
    }

    public Encoder getEncoderRight() {
        return this.encoderRight;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                this.encoderLeft.getRate(),
                this.encoderRight.getRate());
    }

    public double getAverageDistance() {
        return (this.encoderLeft.getDistance() + this.encoderRight.getDistance()) / 2.0;
    }

    public void debugSmartDashboard(boolean isDebugging) {
        if (isDebugging) {
            SmartDashboard.putNumber("[DRIVETRAIN] Encoder Left", this.encoderLeft.get());
            SmartDashboard.putNumber("[DRIVETRAIN] Encoder Right", this.encoderRight.get());
            SmartDashboard.putNumber("[DRIVETRAIN] Average Distance", this.getAverageDistance());
        }
    }

    public void setEncodersDistancePerPulse() {
    var wheelCircumferenceMeters = Units.inchesToMeters(DrivetrainConstants.Chassi.wheelRadius) * 2 * Math.PI;

    var distancePerPulse = wheelCircumferenceMeters / DrivetrainConstants.Encoders.pulsesPerRotation;

    this.encoderLeft.setDistancePerPulse(distancePerPulse);
    this.encoderRight.setDistancePerPulse(distancePerPulse);
  }


    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.backLeft.setVoltage(leftVolts);
        this.backRight.setVoltage(rightVolts);
        this.frontLeft.setVoltage(leftVolts);
        this.frontRight.setVoltage(rightVolts);
        System.out.println("LeftVolts " + leftVolts);
        System.out.println("RightVolts " + rightVolts);
    
        this.drive.feed();
      }

   

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

     public Rotation2d getRotation2d() {
    return this.navX.getRotation2d();
  }

  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  public void updateOdometry() {
    this.odometry.update(
        this.getRotation2d(),
        this.encoderLeft.getDistance(),
        this.encoderRight.getDistance());
  }

  public void resetOdometry(Pose2d pose) {
    this.resetEncoders();

    /*
     * CAUTION! MAY NOT WORK YET
     * SEE WHAT WE SHOULD PUT IN PARAMS
     */
    this.odometry.resetPosition(
        this.getRotation2d(),
        this.getEncoderLeft().getDistance(),
        this.getEncoderRight().getDistance(),
        pose);
  }
}
