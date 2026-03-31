package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {
  public static final class Constants {

    public static final class mainShooterConstants {
      public static final int SHOOTER_LEAD_MOTOR_ID = 6;

      public static final int SHOOTER_FOLLOW1_MOTOR_ID = 7;
      public static final int SHOOTER_FOLLO2_MOTOR_ID = 8;
      public static final int SHOOTER_FOLLOW3_MOTOR_ID = 9;

      public static final double GEAR_RATIO_SHOOTER_LEAD =
          (14 * 42) / (55 * 18); // gear ratio 14:55 and 42:18

      public static final Slot0Configs SHOOTER_GAINS =
          new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

      // FIX THE INVERTED VALUE LATER

      public static final CurrentLimitsConfigs SHOOTER_CONFIG =
          new CurrentLimitsConfigs().withSupplyCurrentLimit(30.0);

      public static final MotorOutputConfigs SHOOTER_OUTPUT_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);

      public static final MotorOutputConfigs SHOOTER2_OUTPUT_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);

      public static final MotorOutputConfigs SHOOTER3_OUTPUT_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);

      public static final MotorOutputConfigs SHOOTER4_OUTPUT_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);

      public static final MotorOutputConfigs SHOOTER_FOLLWER_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);
    }

    public static final class hoodConstants {
      public static final int HOOD_MOTOR_ID = 10;

      public static final Slot0Configs HOOD_GAINS =
          new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

      public static final CurrentLimitsConfigs HOOD_CONFIG =
          new CurrentLimitsConfigs().withSupplyCurrentLimit(30.0);

      public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS =
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake);
    }
  }
}
