package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs {
        public boolean indexerMotorConnected = false;
        public double indexerVelocityRPS = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerVoltage = 0.0;
        
    }
    public default void updateInputs(IndexerIOInputs inputs){}
    public default void setIndexerVoltage(double volts){}
    public default void setIndexerOpenLoop(double input){}
    public default void setIndexerVelocity(double velocity){}
    public default void stop(){}
    
} 