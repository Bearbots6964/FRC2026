// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeConstants.deployMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants.intakeMotorConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Identifiable;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Identifiable {

    private final IntakeIO io;
    // will compile after run
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //Alert for intake motor disconnection
    private final Alert intakeMotorDisconnectedAlert;
    //Alert for deploy motor disconnection
    private final Alert deployMotorDisconnectedAlert;
    //triggers: prevent subsystems from knowing each other to reduce dependency
    @AutoLogOutput
    public final Trigger isDeployed = new Trigger(
        //trigger that is true when the intake is fully deployed
        () -> Math.abs(inputs.deployMotorPositionDegrees
            - IntakeConstants.DEPLOYED_ANGLE.in(Degrees))
            <= IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));
    @AutoLogOutput
    public final Trigger isRetracted = new Trigger(
        //trigger that is true when the intake is fully retracted
        () -> Math.abs(inputs.deployMotorPositionDegrees
            - IntakeConstants.RETRACTED_ANGLE.in(Degrees))
            <= IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));

    public final Trigger isDeployMotorDisconnected = new Trigger(
        () -> !inputs.deployMotorConnected);

    public final Trigger isIntakeMotorDisconnected = new Trigger(
        () -> !inputs.intakeMotorConnected);

    @AutoLogOutput
    private IntakeGoal goal = IntakeGoal.IDLE;

    private final SysIdRoutine routine;

    /**
     * Creates a new Intake.
     */
    public Intake(IntakeIO intakeIO) {
        this.io = intakeIO;
        intakeMotorDisconnectedAlert =
            new Alert("Intake motor disconnected.", Alert.AlertType.kError);
        //critical alert for motor disconnection
        deployMotorDisconnectedAlert =
            new Alert("Deploy motor disconnected.", Alert.AlertType.kError);
        //critical alert for motor disconnection

        routine = new SysIdRoutine(
            new Config(Volts.per(Second).of(0.5), Volts.of(3), null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(io::setDeployVoltage, null, this)
        );

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run, every 20ms by default
        //update the inputs from the IO and log them
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        //check for intake motor disconnection and display if true
        intakeMotorDisconnectedAlert.set(!inputs.intakeMotorConnected);
        deployMotorDisconnectedAlert.set(!inputs.deployMotorConnected);

        //end of periodic
    }

    //command to deploy the intake
    public Command deploy() {
        //run the deploy motor at the specified voltage when this command is scheduled, and stop it when the command ends
        return runOnce(
            () -> io.setDeployPosition(IntakeConstants.DEPLOYED_ANGLE)
            //deploy at the specified voltage
        );
    }

    public Command retract() {
        //run the deploy motor in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
        return runOnce(
            () -> io.setDeployPositionGainSlotTwo(IntakeConstants.RETRACTED_ANGLE)
        );
    }

    public Command tilt() {
        //run the deploy motor in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
        return runOnce(
            () -> io.setDeployPositionGainSlotTwo(IntakeConstants.TILT_ANGLE)
        );
    }

    //command to intake fuel
    public Command intake() {
        //run the intake at the specified voltage when this command is scheduled, and stop it when the command ends
        return runEnd(
            () -> io.setIntakeVoltage(intakeMotorConstants.intakeVoltage),
            () -> io.setIntakeVoltage(0.0)
        );
    }

    public Command autoIntake() {
        return
            runOnce(() -> goal = IntakeGoal.DEPLOY).andThen(
                runOnce(() -> {
                    io.setDeployPosition(IntakeConstants.DEPLOYED_ANGLE);
                    io.setIntakeVoltage(
                        intakeMotorConstants.intakeVoltage);
                }).repeatedly());
    }

    public Command eject() {
        //run the intake in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
        return runEnd(
            () -> io.setIntakeVoltage(-intakeMotorConstants.intakeVoltage),
            //eject at the specified voltage
            () -> io.setIntakeVoltage(0.0)
        );
    }

    public void stopIntake() {
        io.stopIntake();
    }

    public void stopDeploy() {
        io.stopDeploy();
    }

    public Command setGoal(IntakeGoal goal) {
        return defer(() -> {
            Commands.none();
            Command toSchedule = switch (goal) {
                case STOW -> retract().andThen(intake());
                case DEPLOY -> deploy().andThen(intake());
                case TILT -> tilt().andThen(intake());
                case IDLE -> Commands.run(this::stopIntake);
                case EJECT -> deploy().andThen(eject());
            };
            return toSchedule;
        });
    }

    public static enum IntakeGoal {
        /**
         * Stow the intake inside frame perimeter.
         */
        STOW,
        /**
         * Deploy the intake and run the intake motor to collect fuel.
         */
        DEPLOY,
        /**
         * Tilt the intake back to assist with funneling balls into the indexer.
         */
        TILT,
        /**
         * Stop the intake and hold position.
         */
        IDLE,
        /**
         * Reverse the intake and spit out balls.
         */
        EJECT

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return routine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return routine.dynamic(dir);

    }

//end of class
}
