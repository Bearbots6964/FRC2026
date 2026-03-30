package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  public static final class IntakeConsts {
    public static final int INTAKE_LEAD_ID = 15;
    public static final int INTAKE_FOLLOWER_ID = 16;

    public static final double TARGET_RPS = 25.0;

    public static final Slot0Configs INTAKE_CONFIGS =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);

    public static final CurrentLimitsConfigs INTAKE_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30.0);

    public static final MotorOutputConfigs INTAKE_MOTOR_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // check with parker
            .withNeutralMode(NeutralModeValue.Brake);

    public static final MotorOutputConfigs INTAKE_MOTOR_OUTPUT_CONFIGS2 =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // check with parker
            .withNeutralMode(NeutralModeValue.Brake);
  }

  public static final class intakeDeploy {
    public static final int INTAKE_DEPLOY_ID = 17;

    public static final Slot0Configs INTAKE_DEPLOY_SLOT0_CONFIGS =
        new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0).withKV(0.0);
    public static final CurrentLimitsConfigs INTAKE_DEPLOY_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30.0);

    public static final MotorOutputConfigs INTAKE_DEPLOY_OUTPUT_CONFIGS =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // ask parker
            .withNeutralMode(NeutralModeValue.Brake);
  }
}
