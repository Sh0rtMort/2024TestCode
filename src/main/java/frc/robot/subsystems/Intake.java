package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private TalonFX pivotMotor = new TalonFX(0);
    private TalonFX powerMotor = new TalonFX(1);

    // private XboxController gamepad;

    private PIDController pivotPidController = new PIDController(0.2, 0.002, 0);

    private PeriodicIO periodicIO;

    private static class PeriodicIO {
        // Automated control
        PivotStates pivot_target = PivotStates.stow;
        PowerStates intake_state = PowerStates.none;
    
        // Manual control
        double intake_pivot_power = 0.0;
        double intake_power = 0.0;
      }

    public Intake() {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        powerMotor.setNeutralMode(NeutralModeValue.Brake);


        periodicIO = new PeriodicIO();
    }


    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public double getPivotAngleDegrees() {

        return Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    }

    public void zeroPivotPostion() {
        pivotMotor.setPosition(0);
    }

    public enum PivotStates {
        none,
        ground,
        source,
        stow
    }

    public enum PowerStates {
        none, 
        intake,
        eject,
        feedShooter
    }

    @Override
    public void periodic() {
        double pivotAngle = pivotTargetToAngle(periodicIO.pivot_target);
        periodicIO.intake_pivot_power = pivotPidController.calculate(getPivotAngleDegrees(), pivotAngle);
        periodicIO.intake_power = stateToPowerSpeed(periodicIO.intake_state);


        
        outputTelemetry();
    }


    public double pivotTargetToAngle(PivotStates target) {
        switch (target) {
            case ground:
                return 60;
            case source:
                return 190;
            case stow:
                return 270;
            default:
                return 180;
        } 
    } 

    public double stateToPowerSpeed(PowerStates state) {
        switch (state) {
            case intake:
                return 0.6;
            case eject:
                return -0.4;
            case feedShooter:
                return -0.35;
            default: 
                return 0;
        } 
    }

    public void goToGround() {
        periodicIO.pivot_target = PivotStates.ground;
    }

    public void goToSource() {
        periodicIO.pivot_target = PivotStates.source;
    }

    public void goToStow() {
        periodicIO.pivot_target = PivotStates.stow;
    }

    public void setSpeedIntake() {
        periodicIO.intake_state = PowerStates.intake;
    }

    public void setSpeedEject() {
        periodicIO.intake_state = PowerStates.eject;
    }

    public void setSpeedFeedShooter() {
        periodicIO.intake_state = PowerStates.feedShooter;
    }

    public void stopIntake() {
        periodicIO.intake_state = PowerStates.none;
        periodicIO.intake_power = 0;
    }

    public void setPowerState(PowerStates state) {

    }

    public void setPivotTarget(PivotStates target) {
        periodicIO.pivot_target = target;
    }

    public boolean limitSwitchTriggered() {
        return false;
        // return !intakeLimitSwitch.get();
    }

    public void autoStore() {
        //edit this when we add a limit switch
        if (periodicIO.pivot_target == PivotStates.ground && limitSwitchTriggered() && isPivotAtTarget()) {
            periodicIO.pivot_target = PivotStates.stow;
        }
    }

    private boolean isPivotAtTarget() {
        return Math.abs(getPivotPosition() - pivotTargetToAngle(periodicIO.pivot_target)) < 5;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake speed:", stateToPowerSpeed(periodicIO.intake_state));
        SmartDashboard.putNumber("Pivot Abs Enc (get):", pivotMotor.getPosition().getValue());
        SmartDashboard.putNumber("Pivot Abs Enc (getAbsolutePosition):", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
        SmartDashboard.putNumber("Pivot Setpoint:", pivotTargetToAngle(periodicIO.pivot_target));

        SmartDashboard.putNumber("Pivot Power:", periodicIO.intake_pivot_power);
        // SmartDashboard.putNumber("Pivot Current:", pivotMotor.getStatorCurrent());

        SmartDashboard.putBoolean("Intake Limit Switch:", limitSwitchTriggered());
  }


}
