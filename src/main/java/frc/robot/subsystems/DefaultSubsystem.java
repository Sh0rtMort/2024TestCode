// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DefaultSubsystem extends SubsystemBase {
  
  private TalonFX motor1 = new TalonFX(31);
  private TalonFX motor2 = new TalonFX(22);

  public DefaultSubsystem() {
    motor1.setInverted(false);
    motor2.setInverted(true);

    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);

    PIDController shootingPIDController = new PIDController(0.2, 0.002, 0);

    var slot0Configs = new Slot0Configs();
      slot0Configs.kV = 0.12;
      slot0Configs.kP = 0.11;
      slot0Configs.kI = 0.48;
      slot0Configs.kD = 0.01;
      motor1.getConfigurator().apply(slot0Configs, 0.050);
      motor2.getConfigurator().apply(slot0Configs, 0.050);
  }


  public Command RunMotors(double speed) {
    return run(
        () -> {
         setMotorSpeed(speed);
        });
  }

  public Command RunMotorVoltage(double rpm) {
    return run(() -> {
      setMotorVolts(RPMToVolts(rpm));
    });
  }

  public void setMotorSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void setSplitMotorSpeed(double one, double two) {
    motor1.set(one);
    motor2.set(two);
  }

  public void setMotorVolts(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  public void stopMotors() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  public double getMotor1Encoder() {
    return 0;
  }

  public double getMotor2Encoder() {
    return 0;
  }

  public StatusSignal<Double> getVelocity() {
    return motor1.getVelocity();
  }

  public double RPMToVolts(double TargetRPM) {
    //Formula: VConstant = (AppliedVolts / VelocityatVolts) 
    double velocityConstant = 12 / 4500; //Sample Numbers -> vc = 0.002667
    return velocityConstant * TargetRPM;
  }

  @Override
  public void periodic() {

  }
}
