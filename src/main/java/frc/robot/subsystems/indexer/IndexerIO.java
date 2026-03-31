package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    public boolean indexerConnected = false;
    public boolean indexerFollowerConnected = false;

    public Voltage indexerVolts = Volts.zero();

    /*
     * public AngularVelocity indexerSpeed = RotationsPerSecond.zero();
     * public AngularAcceleration indexerAccel = RotationsPerSecondPerSecond.zero();
     * public Current indexerAmps = Amps.zero();
     */
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void stop() {}

  public default void runIndexerOpenLoop() {}
}
