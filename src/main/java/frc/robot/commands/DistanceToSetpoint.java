package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class DistanceToSetpoint extends Command {
  private Shooter shooter;
  private Vision vision;
  private PIDController pidController = new PIDController(0.015, 0.0015, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DistanceToSetpoint(Shooter shooter, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    
    addRequirements(shooter);
  }


  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = Units.degreesToRotations(vision.getPitchOfShootingTarget());
    double speed = pidController.calculate(shooter.getPivotEncoder(), setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPivotMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
