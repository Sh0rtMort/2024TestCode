// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DefaultSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetpointCommand extends Command {
  
  private final DefaultSubsystem m_subsystem;
  
  private PIDController pidController = new PIDController(0, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetpointCommand(DefaultSubsystem subsystem, double setpoint) {
    m_subsystem = subsystem;
    addRequirements(subsystem);

    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0);
  }

  
  @Override
  public void initialize() {

  }

  
  @Override
  public void execute() {
    double speed1 = pidController.calculate(m_subsystem.getMotor1Encoder());
    double speed2 = pidController.calculate(m_subsystem.getMotor2Encoder());

    m_subsystem.setSplitMotorSpeed(speed1, speed2);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotors();
  }

}
