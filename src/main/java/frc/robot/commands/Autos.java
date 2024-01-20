// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DefaultSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DefaultSubsystem subsystem) {
    return Commands.run(()->subsystem.setMotorSpeed(0.2));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
