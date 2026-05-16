package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Alert motorDisconnected;
    private final Alert overcurrentAlert;
    private final Trigger overcurrentTrigger;
    private final Debouncer overcurrentDebouncer;

    @Setter
    @AutoLogOutput
    private IndexerGoal goal = IndexerGoal.IDLE;

    private final IndexerVisualizer visualizer;

    private final DoubleSupplier speedSupplier;
    private final LoggedTunableNumber tuningIndexerSpeed = new LoggedTunableNumber(
        "Indexer/Tuning/SpeedRPS", 0.0);

    public Indexer(IndexerIO io, DoubleSupplier speedSupplier) {
        this.io = io;
        this.speedSupplier = speedSupplier;

        motorDisconnected = new Alert("Indexer motor disconnected", AlertType.kError);
        overcurrentDebouncer = new Debouncer(0.2);
        overcurrentTrigger = new Trigger(
            () -> overcurrentDebouncer.calculate(inputs.indexerCurrentAmps > 90.0));
//        overcurrentTrigger.onTrue(runIndexer());
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
        var speed = speedSupplier.getAsDouble();
        Logger.recordOutput("Indexer/TargetSpeed", speed);

        if (goal == IndexerGoal.IDLE)
            io.stop();
        else if (goal == IndexerGoal.ACTIVE)
            io.setIndexerVelocity(speed);
        else if (goal == IndexerGoal.REVERSE)
            io.setIndexerOpenLoop(-IndexerConstants.MOTOR_SPEED_PERCENTAGE);
    }

    public Command setGoalCommand(IndexerGoal goal) {
        return runOnce(() -> this.goal = goal);
    }

    public Command runEndIndexer() {
        return runEnd(() -> goal = IndexerGoal.ACTIVE,
            () -> goal = IndexerGoal.IDLE).finallyDo(
            () -> goal = IndexerGoal.IDLE
        );
    }

    public Command stopIndexer() {
        return runOnce(() -> {io.stop(); goal = IndexerGoal.IDLE;});
    }


    public enum IndexerGoal {
        ACTIVE,
        IDLE,
        REVERSE
    }

}
