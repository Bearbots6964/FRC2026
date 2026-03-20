package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Alert motorDisconnected;
    private final Alert overcurrentAlert;
    private final Trigger overcurrentTrigger;
    private final Debouncer overcurrentDebouncer;

    @AutoLogOutput
    private IndexerGoal goal = IndexerGoal.IDLE;

    private final IndexerVisualizer visualizer;

    public Indexer(IndexerIO io) {
        this.io = io;

        motorDisconnected = new Alert("Indexer motor disconnected", AlertType.kError);
        overcurrentDebouncer = new Debouncer(0.2);
        overcurrentTrigger = new Trigger(() -> overcurrentDebouncer.calculate(inputs.indexerCurrentAmps > 90.0));
        overcurrentTrigger.onTrue(runIndexer());
        overcurrentAlert = new Alert("Indexer hitting current limit!", AlertType.kWarning);
        visualizer = new IndexerVisualizer();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Indexer", inputs);

        motorDisconnected.set(!inputs.indexerMotorConnected);

        visualizer.update(Units.RotationsPerSecond.of(inputs.indexerVelocityRPS));
        if (DriverStation.isDisabled()) {
            goal = IndexerGoal.IDLE;
            io.stop();
        }
    }

    public Command setGoal(IndexerGoal goal) {
        return defer(() -> {
            Command toSchedule = Commands.none();
            if (goal == IndexerGoal.ACTIVE && this.goal != IndexerGoal.ACTIVE) {
                toSchedule = runIndexer();
            } else if (goal == IndexerGoal.IDLE) {
                toSchedule = stopIndexer();
            }
            this.goal = goal;
            return toSchedule;
        });
    }

    public Command runIndexer() {
        return runOnce(() -> io.setIndexerOpenLoop(-IndexerConstants.AGITATE_SPEED_PERCENTAGE))
            .andThen(Commands.waitSeconds(IndexerConstants.AGITATE_DURATION_SECONDS))
            .andThen(runOnce(() -> io.setIndexerOpenLoop(IndexerConstants.MOTOR_SPEED_PERCENTAGE)));
    }

    public Command runStopIndexer() {
        return runOnce(() -> io.setIndexerOpenLoop(-IndexerConstants.AGITATE_SPEED_PERCENTAGE))
            .andThen(Commands.waitSeconds(IndexerConstants.AGITATE_DURATION_SECONDS))
            .andThen(runOnce(() -> io.setIndexerOpenLoop(IndexerConstants.MOTOR_SPEED_PERCENTAGE)).repeatedly());
    }

    public Command stopIndexer() {
        return runOnce(io::stop);
    }


    public enum IndexerGoal {
        ACTIVE,
        IDLE
    }

}
