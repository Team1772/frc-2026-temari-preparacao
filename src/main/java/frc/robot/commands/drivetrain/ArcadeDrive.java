package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArcadeDrive extends Command {
  private Drivetrain drivetrain;
  private DoubleSupplier forward;
  private DoubleSupplier rotation;
  private static final Double SPEED_LIMITER = 0.8;
  private static final Double ROTATION_LIMITER = 0.6;
  private static final boolean IS_INVERT_ROBOT_DRIVE = false;

  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.rotation = rotation;

    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    var speed = (IS_INVERT_ROBOT_DRIVE ? -this.forward.getAsDouble() : this.forward.getAsDouble()) * SPEED_LIMITER;
    var rotation = (IS_INVERT_ROBOT_DRIVE ? -this.rotation.getAsDouble() : this.rotation.getAsDouble()) * ROTATION_LIMITER;
    this.drivetrain.arcadeDrive(speed, rotation);
  }
}