package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final Alert motorDisconnected;

    public Indexer(IndexerIO io) {
        this.io = io;

        motorDisconnected = new Alert("Indexer motor disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Indexer", inputs);

        motorDisconnected.set(!inputs.indexerMotorConnected);
    }

    public Command runIndexerOpenLoopCommand(double input) {
        return runEnd(
                () -> io.setIndexerOpenLoop(input * IndexerConstants.MOTOR_SPEED_PERCENTAGE),
                () -> io.stop());
    }

    public Command runIndexerVelocityCommand(double velocity) {
        return runEnd(
                () -> io.setIndexerVelocity(velocity),
                () -> io.stop()
        );

    }

    public Command runIndexerVoltageCommand(double volts) {
        return runEnd(
                () -> io.setIndexerVoltage(volts),
                () -> io.stop());
    }
}
