// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.Swerve;
import frc.robot.commands.AimThenShoot;
import frc.robot.commands.Autos;
import frc.robot.commands.AimAtShootingTarget;
import frc.robot.commands.IntakePivot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID;


public class RobotContainer {
 
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Vision vision = new Vision();
  private final Swerve s_swerve = new Swerve();



  // private final CommandXboxController m_driverController =
  //       new CommandXboxController(0);

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;      

  private final boolean robotCentric = m_driverController.y().getAsBoolean();
  // private final JoystickButton robotCentric = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);


  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    s_swerve.setDefaultCommand(
            new TeleopSwerve(
                s_swerve, 
                () -> -m_driverController.getRawAxis(translationAxis), 
                () -> -m_driverController.getRawAxis(strafeAxis), 
                () -> -m_driverController.getRawAxis(rotationAxis), 
                () -> robotCentric
            )
        );
    
    
    configureBindings();
    intake.zeroPivotPostion();


    // intake.writePeriodicOutputs();
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
  private double ground = 60;
  private double store = 210;
  private double source = 190;

  private void configureBindings() {
    
    m_driverController.start().onTrue(new InstantCommand(() -> s_swerve.zeroHeading()));

    m_driverController.a().toggleOnTrue(new IntakePivot(intake, ground));
    m_driverController.b().toggleOnTrue(new IntakePivot(intake, source));
    m_driverController.x().toggleOnTrue(new IntakePivot(intake, store));
    

    m_driverController.leftBumper().whileTrue(shooter.RunMotors(0.2));
    m_driverController.rightBumper().whileTrue(shooter.RunMotorVoltage(400));
    m_driverController.back().whileTrue(new AimAtShootingTarget(shooter, vision));
    m_driverController.rightTrigger().whileTrue(new AimThenShoot(vision, intake, shooter));
    // m_driverController.x().onTrue(new SetpointCommand(shooter, 0));
  }
  
  public Command getAutonomousCommand() {
    
    return Autos.exampleAuto(shooter);
  }
}
