package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArcadeDrive extends Command {
  private Drivetrain drivetrain;
  private DoubleSupplier forward;
  private DoubleSupplier rotation;
  private static final Double SPEED_LIMITER = 1.0;
  private static final Double ROTATION_LIMITER = 1.0;

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
    this.drivetrain.arcadeDrive(this.forward.getAsDouble() * SPEED_LIMITER, this.rotation.getAsDouble() * ROTATION_LIMITER);
  }
}