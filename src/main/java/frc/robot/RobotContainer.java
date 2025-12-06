package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomous.DriveForwardAuto;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.core.util.TrajectoryBuilder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
  public final Drivetrain drivetrain = new Drivetrain();
  public TrajectoryBuilder trajectoryBuilder;
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    trajectoryBuilder = new TrajectoryBuilder(drivetrain, "1-forward");
    configureBindings();
  }
  
  private void configureBindings() {
        buttonBindingsSysId();
        
        this.drivetrain.setDefaultCommand(
        new ArcadeDrive(
            this.drivetrain,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getRightX()
            ));
  
  }

    public void buttonBindingsSysId() {
    if (DrivetrainConstants.SysId.isSysIdTunning) {
      this.m_driverController.a().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      this.m_driverController.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      this.m_driverController.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      this.m_driverController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }
  }

  public Command getAutonomousCommand() {
    return new DriveForwardAuto(drivetrain, trajectoryBuilder);
  }
}
