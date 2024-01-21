package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AimAtShootingTarget extends Command {
  private Shooter shooter;
  private Vision vision;
  private PIDController pidController = new PIDController(0.015, 0.0015, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimAtShootingTarget(Shooter shooter, Vision vision) {
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
    //TODO:Add adjusted yaw with swerve
    
    // double setpoint = Units.degreesToRotations(vision.getPitchOfShootingTarget());

    //setpoint is tracked in degrees due to getPitch being returned in degrees
    double setpoint = vision.getPitchOfShootingTarget();
    //speed calculator must be calculated using the encoder ticks converted to degrees. this makes the units constant between all variables
    double speed = pidController.calculate(Conversions.falconToDegrees(shooter.getPivotEncoder(), 1), setpoint);

    shooter.setPivotMotor(speed);
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
