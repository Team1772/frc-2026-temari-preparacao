package frc.core.util.sysId;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DrivetrainSysIdTuning {

  private final MutVoltage appliedVoltage = Volts.mutable(0);
  private final MutDistance measureDistance = Meters.mutable(0);
  private final MutLinearVelocity measureVelocity = MetersPerSecond.mutable(0);

  private SysIdRoutine sysIdRoutine;
  private WPI_TalonSRX motorLeftBack;
  private WPI_VictorSPX motorRightBack;
  private WPI_TalonSRX motorLeftFront;
  private WPI_TalonSRX motorRightFront;
  private Encoder encoderLeft;
  private Encoder encoderRight;
  private Drivetrain drivetrain;

  public DrivetrainSysIdTuning(Drivetrain drivetrain) {
    this.motorLeftBack = drivetrain.getMotorLeftBack();
    this.motorLeftFront = drivetrain.getMotorLeftFront();
    this.motorRightBack = drivetrain.getMotorRightBack();
    this.motorRightFront = drivetrain.getMotorRightFront();
    this.encoderLeft = drivetrain.getEncoderLeft();
    this.encoderRight = drivetrain.getEncoderRight();
    this.drivetrain = drivetrain;

  }

  public void enable() {

    this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (volts) -> {
              motorLeftBack.setVoltage(volts);
              motorLeftFront.setVoltage(volts);
              motorRightBack.setVoltage(volts);
              motorRightFront.setVoltage(volts);
            },

            log -> {
              logMotor(log, "drive-leftBack", motorLeftBack, encoderLeft);
              logMotor(log, "drive-leftFront", motorLeftFront, encoderLeft);
              logMotor(log, "drive-rightBack", motorRightBack, encoderRight);
              logMotor(log, "drive-rightFront", motorRightFront, encoderRight);
            },
            this.drivetrain));
  }

  public void logMotor(SysIdRoutineLog log, String motorName, WPI_TalonSRX motor, Encoder encoder) {
    log.motor(motorName)
        .voltage(
            appliedVoltage.mut_replace(
                motor.get()
                    * RobotController
                        .getBatteryVoltage(),
                Volts))
        .linearPosition(measureDistance.mut_replace(
            encoder.getDistance(),
            Meters))
        .linearVelocity(
            measureVelocity.mut_replace(
                encoder.getRate(),
                MetersPerSecond));
  }

  public void logMotor(SysIdRoutineLog log, String motorName, WPI_VictorSPX motor, Encoder encoder) {
    log.motor(motorName)
        .voltage(
            appliedVoltage.mut_replace(
                motor.get()
                    * RobotController
                        .getBatteryVoltage(),
                Volts))
        .linearPosition(measureDistance.mut_replace(
            encoder.getDistance(),
            Meters))
        .linearVelocity(
            measureVelocity.mut_replace(
                encoder.getRate(),
                MetersPerSecond));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

}