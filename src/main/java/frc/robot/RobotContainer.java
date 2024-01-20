// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SetpointCommand;
import frc.robot.subsystems.DefaultSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
 
  private final DefaultSubsystem m_exampleSubsystem = new DefaultSubsystem();
  private final Intake intake = new Intake();

  
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (m_driverController.a().getAsBoolean()) {
      intake.goToGround();
    } else if (m_driverController.b().getAsBoolean()) {
      intake.goToSource();
    } else if (m_driverController.x().getAsBoolean()) {
      intake.goToStow();
    } else {
      intake.stopIntake();
    }

    
    m_driverController.b().whileTrue(m_exampleSubsystem.RunMotors(0.2));
    m_driverController.a().whileTrue(m_exampleSubsystem.RunMotorVoltage(400));
    m_driverController.x().onTrue(new SetpointCommand(m_exampleSubsystem, 0));
  }

  
  public Command getAutonomousCommand() {
    
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
