package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivot extends Command {
  private final Intake intake;
  private PIDController pidController = new PIDController(0.015, 0.00015, 0);
  private double setpoint;

  
  public IntakePivot(Intake intake, double setpoint) {
    this.intake = intake;
    this.setpoint = setpoint;
    addRequirements(intake);

    //setpoint is tracked in degrees here
    pidController.setSetpoint(setpoint);

    pidController.setTolerance(0);

  }

  @Override
  public void initialize() {
    System.out.println("Moving to point:" + setpoint);
  }

  @Override
  public void execute() {
    //since setpoint is in degrees, we need to track the encoder in terms of degrees, not units
    double speed = pidController.calculate(intake.getPivotAngleToFalconDegrees(), setpoint);
    intake.setPivotMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    System.out.println("Command Stopped");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
