package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private STATE goal = STATE.OFF;

  private final Alert indexerAlert;
  private final Alert followerAlert;

  public Indexer(IndexerIO io) {
    this.io = io;

    indexerAlert = new Alert("Lead indexer motor disconnected", AlertType.kError);
    followerAlert = new Alert("Follower indexer motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    indexerAlert.set(!inputs.indexerConnected);
    followerAlert.set(!inputs.indexerFollowerConnected);
  }

  public Command setStateCommand(STATE goal) {
    return runOnce(
        () -> {
          this.goal = goal;
          switch (goal) {
            case ON:
              startIndexer();
              break;
            case OFF:
              stop();
              break;
            default:
              stop();
              break;
          }
        });
  }

  public void startIndexer() {
    io.runIndexerOpenLoop();
  }

  public void stop() {
    io.stop();
  }

  public enum STATE {
    OFF,
    ON
  }
}
