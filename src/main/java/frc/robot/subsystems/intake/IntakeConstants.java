// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * Add your docs here.
 */
public class IntakeConstants {

    // Angles measured from robot, stored as Rotation2d(x, y) vector components
    // DEPLOYED_ANGLE ≈ 13.45°, RETRACTED_ANGLE ≈ 86.67°
    public static final Angle DEPLOYED_ANGLE = Degrees.of(-10.2232050549);
    public static final Angle RETRACTED_ANGLE = Degrees.of(91.758);
    public static final Angle TILT_ANGLE = Degrees.of(60.0);
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(0.5);

    public static final Angle START_ANGLE = Degrees.of(91.758);

    public static final class intakeMotorConstants {

        public static final int intakeMotorCanID = 2;
        public static final int intakeFollowerMotorCanID = 3;
        public static final double gearRatio = 3.0; //gear ratio is 5:1
        public static final double intakeVoltage = 11.0;

        public static final Slot0Configs intakeMotorGains = new Slot0Configs()
            .withKP(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0);

        public static final CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(70.0);

        public static final MotorOutputConfigs intakeMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    }


    public static final class deployMotorConstants {
        public static final int deployMotorCanID = 1;
        public static final double gearRatio = 4.0 * 4.0 * 3.0; //gear ratio is 4:1, 3:1, 3:1
        public static final double deployVoltage = 6.0;
        public static final double deployTimeoutSeconds = 5.0;
        public static final double retractTimeoutSeconds = 5.0;
        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A9.781%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A10.7756464614%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X44%20%28FOC%29%22%7D&ratio=%7B%22magnitude%22%3A48%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final Slot0Configs deployMotorGains = new Slot0Configs()
            .withKP(80.0)
            .withKD(4.0)
            .withKS(0.0)
            .withKV(4.69)
            .withKA(0.10)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);
        public static final Slot1Configs retractGains = new Slot1Configs()
            .withKP(60.0)
            .withKD(2.0)
            .withKS(0.5)
            .withKV(4.69)
            .withKA(0.10)
            .withKG(10.0)
            .withGravityArmPositionOffset(RETRACTED_ANGLE.minus(Degrees.of(81.123)))
            .withGravityType(GravityTypeValue.Arm_Cosine);


        public static final CurrentLimitsConfigs deployCurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120.0);

        public static final MotorOutputConfigs deployMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs deployFeedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(gearRatio);

        public static final SoftwareLimitSwitchConfigs deploySoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(RETRACTED_ANGLE)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(DEPLOYED_ANGLE);

        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(1.0);
    }

}
