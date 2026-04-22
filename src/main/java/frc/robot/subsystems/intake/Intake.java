// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeConstants.DeployMode;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.IntakeConstants.intakeMotorConstants;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Identifiable;
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

    @AutoLogOutput
    private DeployMode deployMode = DeployMode.STOW;

    @AutoLogOutput
    private IntakeMode intakeMode = IntakeMode.STOP;

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

//        routine = new SysIdRoutine(
//            new Config(Volts.per(Second).of(0.5), Volts.of(3), null,
//                (state) -> SignalLogger.writeString("state", state.toString())),
//            new Mechanism(io::setDeployVoltage, null, this)
//        );

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

        io.setIntakeMotorVelocity(RotationsPerSecond.of(IntakeConstants.intakeMap.get(intakeMode)));
        io.setDeployPosition(Degrees.of(IntakeConstants.deployMap.get(deployMode)));
        //end of periodic
    }

    public Command tilt() {
        return Commands.waitSeconds(1.25)
            .andThen(runOnce(() -> goal = IntakeGoal.STOW));
    }


    public Command autoIntake() {
        return
            run(() -> goal = IntakeGoal.DEPLOY);
    }

    public Command setGoalCommand(IntakeGoal goal) {
        return runOnce(() -> setGoal(goal));
    }

    public void setGoal(IntakeGoal goal) {
        this.goal = goal;
        switch (goal) {
            case STOW -> {
                intakeMode = IntakeMode.RUN_SLOW;
                deployMode = DeployMode.STOW;
            }
            case DEPLOY -> {
                intakeMode = IntakeMode.RUN;
                deployMode = DeployMode.DEPLOY;
            }
            case TILT -> {
                intakeMode = IntakeMode.RUN_SLOW;
                deployMode = DeployMode.TILT;
            }
            case IDLE -> {
                intakeMode = IntakeMode.STOP;
            }
            case EJECT -> {
                intakeMode = IntakeMode.REVERSE;
            }
        }
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
