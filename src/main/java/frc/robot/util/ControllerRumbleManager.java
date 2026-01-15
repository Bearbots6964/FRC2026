package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerPreferences;
import java.util.Set;
import java.util.function.DoubleConsumer;

public class ControllerRumbleManager {

    /*
    Match Timing
    Once auto ends and teleop starts:
    - 10 seconds of both hubs active
    - 25 seconds of the hub of whatever team got less points in auto
    - 25 seconds of the opposite team's hub being active
    - Repeat the last two steps
    - 30 seconds of endgame
    (total 2:20)
     */
    private final DoubleConsumer rumble;
    private Alliance firstAlliance;
    private Alliance ourAlliance;

    private boolean initialized = false;

    /**
     * Constructor for ControllerRumbleManager, which manages rumble feedback for controllers during
     * a match.
     *
     * @param rumble A DoubleConsumer that sets the rumble intensity on the controller.
     */
    public ControllerRumbleManager(DoubleConsumer rumble) {
        this.rumble = rumble;
    }

    // Recursive countdown command
    private Command countdown(int iter) {
        // 150 ms of rumble every second for 5 seconds
        return on(0.15, ControllerPreferences.countdownRumbleIntensity).andThen(off(0.85))
            .andThen(iter > 1 ? countdown(iter - 1) : Commands.none());
    }

    // Countdown sequence command
    private Command countdownSequence() {
        return countdown(5).finallyDo(() -> rumble.accept(0.0));
    }

    // Hub active rumble pattern
    private Command hubActive() {
        return on(0.25, ControllerPreferences.hubActiveRumbleIntensity).andThen(off(0.25))
            .andThen(on(0.25, ControllerPreferences.hubActiveRumbleIntensity)).andThen(off(0.25))
            .andThen(on(1.0, ControllerPreferences.hubActiveRumbleIntensity))
            .finallyDo(() -> rumble.accept(0.0));
    }

    // Hub inactive rumble pattern
    private Command hubInactive() {
        return on(0.125, ControllerPreferences.hubInactiveRumbleIntensity).andThen(off(0.125))
            .andThen(on(0.125, ControllerPreferences.hubInactiveRumbleIntensity))
            .andThen(off(0.125))
            .andThen(on(0.125, ControllerPreferences.hubInactiveRumbleIntensity))
            .finallyDo(() -> rumble.accept(0.0));
    }

    // Select the first shift's hub rumble pattern based on alliance
    private Command selectHub() {
        Command c;

        if (ourAlliance == firstAlliance) {
            // our hub is active first
            c = hubActive();
        } else {
            // their hub is active first
            c = hubInactive();
        }

        return c.alongWith(Commands.waitSeconds(20.0)).andThen(countdownSequence());
    }

    // Select the second shift's hub rumble pattern based on alliance
    private Command selectOtherHub() {
        Command c;

        if (ourAlliance != firstAlliance) {
            // our hub is active second
            c = hubActive();
        } else {
            // their hub is active second
            c = hubInactive();
        }

        return c.alongWith(Commands.waitSeconds(20.0)).andThen(countdownSequence());
    }

    // Recursive endgame rumble pattern
    private Command endgame(int iter) {
        return on(0.0625, ControllerPreferences.endgameRumbleIntensity).andThen(off(0.0625))
            .andThen(iter > 1 ? endgame(iter - 1) : Commands.none());
    }

    // Endgame sequence command
    private Command endgameSequence() {
        return endgame(5).alongWith(Commands.waitSeconds(25.0).andThen(countdownSequence()))
            .finallyDo(() -> rumble.accept(0.0));
    }

    private void populateAllianceData() {
        ourAlliance = DriverStation.getAlliance()
            .orElseThrow(() -> new IllegalStateException("Alliance data not available"));
        var gameData = DriverStation.getGameSpecificMessage();
        if (!gameData.isEmpty()) {
            char firstChar = gameData.charAt(0);
            switch (firstChar) {
                case 'R' -> firstAlliance = Alliance.Red;
                case 'B' -> firstAlliance = Alliance.Blue;
                default -> firstAlliance = ourAlliance; // default to our alliance if unknown
            }
        } else {
            firstAlliance = ourAlliance; // default to our alliance if no data
        }

    }

    /**
     * Full match rumble sequence command. Runs for full length of match.
     *
     * @return A Command that executes the full match rumble sequence.
     */
    public Command fullMatchSequence() {
        initialized = true;
        // defer construction of commands requiring knowledge of which alliance is first until the end of the first 10 seconds of teleop
        return Commands.waitSeconds(5.0).andThen(countdownSequence())
            .alongWith( // fetch our alliance
                Commands.runOnce(this::populateAllianceData))
            .andThen(Commands.defer(() -> Commands.sequence(
                    // sequence of match rumble events. dependent on which alliance is first
                    selectHub(), selectOtherHub(), selectHub(), selectOtherHub(), endgameSequence()),
                Set.of()));

    }

    /**
     * Checks if the rumble manager has been initialized and started.
     */
    public boolean isInitialized() {
        return initialized;
    }

    // Rumble on command
    private Command on(double time, double intensity) {
        return Commands.run(() -> rumble.accept(intensity)).withTimeout(time);
    }

    // Rumble off command
    private Command off(double time) {
        return Commands.run(() -> rumble.accept(0.0)).withTimeout(time);
    }
}
