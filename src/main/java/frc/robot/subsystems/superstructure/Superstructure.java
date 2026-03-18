// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;
import frc.robot.util.HubShiftUtil;
import java.util.Map;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.Goal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private final Turret turret;
    private final Intake intake;
    private final Indexer indexer;

    private final Supplier<Pose2d> poseSupplier;

    @Getter
    @AutoLogOutput
    private Goal goal = Goal.SCORING;

    @AutoLogOutput
    public final Trigger inAllianceZoneTrigger = new Trigger(this::inAllianceZone);

    @AutoLogOutput
    private boolean shiftOverride = false;

    /**
     * Will trigger when hub is active, or when there is less than ACTIVE_PRESHOOT_TIME until the next active period
     * 100 seconds is an arbitrarily long time so as to not trigger shooting when timeRemainingInCurrentShift gives Optional.empty()
     */
    @AutoLogOutput
    public final Trigger activeHubTrigger =
        new Trigger(() -> HubShiftUtil.getShiftedShiftInfo(shiftOverride).active());

    @AutoLogOutput
    public final Trigger activeInZoneTrigger =
        inAllianceZoneTrigger.and(DriverStation::isTeleop).and(activeHubTrigger);

    @AutoLogOutput
    public final Trigger inactiveInZoneTrigger =
        inAllianceZoneTrigger.and(DriverStation::isTeleop).and(activeHubTrigger.negate());

    @AutoLogOutput
    public final Trigger leaveZoneTrigger = inAllianceZoneTrigger.negate().and(DriverStation::isTeleop);

    private final Map<Goal, Supplier<Command>> goalCommands;

    /** Creates a new Superstructure. */
    public Superstructure(Turret turret, Intake intake, Indexer indexer, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.intake = intake;
        this.indexer = indexer;
        this.poseSupplier = poseSupplier;

        goalCommands = Map.of(
            Goal.SCORING,
            () -> Commands.sequence(
                    this.turret.setGoal(TurretGoal.SCORING),
                    this.intake.setGoal(IntakeGoal.DEPLOY),
                    this.indexer.setGoal(IndexerGoal.ACTIVE)
                )
                .withName("Start scoring"),
            Goal.PASSING,
            () -> Commands.sequence(
                    this.turret.setGoal(TurretGoal.PASSING).onlyIf(inAllianceZoneTrigger.negate()),
                    this.intake.setGoal(IntakeGoal.DEPLOY),
                    this.indexer.setGoal(IndexerGoal.ACTIVE).onlyIf(inAllianceZoneTrigger.negate())
                )
                .withName("Start passing"),
            Goal.COLLECTING,
            () -> Commands.sequence(
                    this.turret.setGoal(TurretGoal.IDLE),
                    this.intake.setGoal(IntakeGoal.DEPLOY),
                    this.indexer.setGoal(IndexerGoal.IDLE)
                )
                .withName("Start collecting"),
            Goal.EXPANDED,
            () -> Commands.sequence(
                    this.turret.setGoal(TurretGoal.IDLE),
                    this.intake.setGoal(IntakeGoal.IDLE),
                    this.indexer.setGoal(IndexerGoal.IDLE)
                )
                .withName("Start expanded"),
            Goal.IDLE,
            () -> Commands.sequence(
                    this.turret.setGoal(TurretGoal.IDLE),
                    this.intake.setGoal(IntakeGoal.STOW),
                    this.indexer.setGoal(IndexerGoal.IDLE)
                )
                .withName("Idle"));

        activeInZoneTrigger.onTrue(this.setGoal(Goal.SCORING));
        inactiveInZoneTrigger.onTrue(this.setGoal(Goal.COLLECTING));
        leaveZoneTrigger.onTrue(this.setGoal(Goal.PASSING).onlyIf(() -> this.goal != Goal.COLLECTING));

        SmartDashboard.putData("Overrides/Shift", enableShiftOverride());
    }

    private boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue && pose.getMeasureX().lt(Units.Meters.of(LinesVertical.allianceZone).plus(Dimensions.FULL_WIDTH.div(2)))
            || !isBlue
            && pose.getMeasureX()
            .gt(Units.Meters.of(FieldConstants.fieldLength).minus(
                Units.Meters.of(LinesVertical.allianceZone).plus(Dimensions.FULL_WIDTH.div(2))));
    }

    public Command setGoal(Goal newGoal) {
        return this.runOnce(() -> this.goal = newGoal)
            .andThen(goalCommands.get(newGoal).get())
            .withName("Set goal");
    }

    public Command toggleCollecting() {
        return Commands.either(stopCollecting(), this.setGoal(Goal.COLLECTING), () -> this.goal == Goal.COLLECTING);
    }

    /** Handle state logic for transitioning out of COLLECTING */
    public Command stopCollecting() {
        return Commands.either(this.setGoal(Goal.SCORING), this.setGoal(Goal.PASSING), activeInZoneTrigger);
    }

    public Command enableShiftOverride() {
        return Commands.startEnd(() -> shiftOverride = true, () -> shiftOverride = false)
            .withName("Override active first");
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
            "Superstructure/Shift Time", HubShiftUtil.getOfficialShiftInfo().remainingTime());
    }

    /**
     * The high-level goal of the superstructure, which determines the behavior of the turret, intake, and indexer. The
     * goal is automatically set based on the robot's position on the field and the hub's active/inactive state, but can also
     * be manually selected by the driver using the setGoal command as well as the various related commands.
     */
    public static enum Goal {
        /**
         * Score fuel in the hub. Automatically activated when entering the alliance zone if the hub is active.
         */
        SCORING,
        /**
         * Pass fuel to our alliance zone. Automatically activated when leaving the alliance zone, unless currently collecting.
         */
        PASSING,
        /**
         * Collect fuel from the field. Automatically activated when entering the alliance zone if the hub is inactive.
         */
        COLLECTING,
        /**
         * Don't actively do anything, but keep the intake expanded for extra hopper capacity.
         */
        EXPANDED,
        /**
         * Don't actively do anything, and stow the intake. Make sure to activate this before climbing to avoid extension limit penalties.
         */
        IDLE
    }
}
