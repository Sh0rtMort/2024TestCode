package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AimThenShoot extends Command {
  private final Vision vision;
  private final Intake intake;
  private final Shooter shooter;
  
  

  public AimThenShoot(Vision vision, Intake intake, Shooter shooter) {
    this.vision = vision;
    this.intake = intake;
    this.shooter = shooter;
    
    addRequirements(vision);
    addRequirements(intake);
    addRequirements(shooter);
  }


  @Override
  public void initialize() {
    System.out.println("Vision Shooting Enabled");
  }


  @Override
  public void execute() {
    //i dont think this works how i want it to
    new SequentialCommandGroup(
        new ParallelCommandGroup( 
            new RunCommand(() -> shooter.RPMToVolts(3500)), 
            new IntakePivot(intake, 210),
            new DistanceToSetpoint(shooter, vision)//.withTimeout(1)
            ).withTimeout(3),
        new ParallelRaceGroup(
            new RunCommand(() -> shooter.RPMToVolts(3500)),
            new RunCommand(() -> intake.setIntakeMotor(-0.45)).withTimeout(0.3)
        )
    );
  }

  
  @Override
  public void end(boolean interrupted) {
    shooter.setMotorSpeed(0);
    shooter.setPivotMotor(0);
    intake.setIntakeMotor(0);
    System.out.println("Vision Shooting Ended");
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
