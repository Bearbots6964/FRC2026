package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class IndexerConstants {

    public static final double MOTOR_SPEED_PERCENTAGE = 0.3;

    public static final class TalonFXConstants {
        public static final int MOTOR_ID = 17;
        public static final double GEAR_RATIO = (64 / 16) * 3; // 3:1 gearbox with a 16:64 belt reduction
        public static final Slot0Configs MOTOR_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0);

        public static final CurrentLimitsConfigs MOTOR_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLowerLimit(30.0);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

    }

}
