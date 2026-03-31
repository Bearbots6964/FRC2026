package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
  public static final class IndexerConsts {
    public static final int INDEXER_MOTOR_ID = 3;
    public static final int FOLLOWER_INDEXER_MOTOR_ID = 4;

    public static final double SPEED_PERCENTAGE = 0.3;

    public static final CurrentLimitsConfigs INDEXER_LIMITS =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30.0);

    public static final MotorOutputConfigs INDEXER_MOTOR_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs FOLLOWE_MOTOR_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
  }
}
