package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;

public class TurretConstants {

    public static final double turretToleranceDegrees = 2.0;

    public static final class TargetingConstants {
        public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
            Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2).plus(Inches.of(85)), Inches.zero());
        public static final Translation3d PASSING_SPOT_CENTER =
            new Translation3d(Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2), Inches.zero());
        public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
            Inches.of(90), Units.Meters.of(FieldConstants.fieldWidth / 2).minus(Inches.of(85)), Inches.zero());

        public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20.0);

        public static final Distance FLYWHEEL_RADIUS = Inches.of(2.0);

        public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShotData::interpolate);
        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

        static {
            // TODO: Fill these maps with data from testing
//            SHOT_MAP.put(1.0, new ShotData(5, 1));
            SHOT_MAP.put(1.469509, new ShotData(Units.RotationsPerSecond.of(40.0), Turret.actualHoodPositionToAngle(40.0)));
            SHOT_MAP.put(2.005032, new ShotData(Units.RotationsPerSecond.of(45.0), Turret.actualHoodPositionToAngle(45.0)));
            SHOT_MAP.put(2.507583, new ShotData(Units.RotationsPerSecond.of(47.0), Turret.actualHoodPositionToAngle(50.0)));
            SHOT_MAP.put(3.033000, new ShotData(Units.RotationsPerSecond.of(50.0), Turret.actualHoodPositionToAngle(60.0)));
            SHOT_MAP.put(3.504614, new ShotData(Units.RotationsPerSecond.of(53.0), Turret.actualHoodPositionToAngle(70.0)));
            TOF_MAP.put(1.0, 1.0);
        }

        public static final int LOOKAHEAD_ITERATIONS = 2;
    }

    public static final class TalonFXConstants {

        public static final class TurnMotorConstants {
            public static final int TURN_MOTOR_ID = 8;
            public static final int TURN_ENCODER_ID = 9;

            public static final Angle MAX_TURN_ANGLE = Units.Degrees.of(180.0);
            public static final Angle MIN_TURN_ANGLE = Units.Degrees.of(-180.0);

            public static final double MOTOR_TO_TURRET_RATIO = 4.8
                * 10.0; // 4.8:1 gearing from motor to encoder, 10:1 gearing from encoder to turret
            public static final double ENCODER_TO_TURRET_RATIO = -10.0; // 10:1 gearing from encoder to turret

            public static final Slot0Configs TURN_GAINS = new Slot0Configs()
                .withKP(59.757)
                .withKD(10.987)
                .withKS(0.355)
                .withKV(5.2411)
                .withKA(1.6317);

            public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLowerLimit(30.0);

            public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            public static final double MAX_OUTPUT_VOLTS = 2.0;
        }

        public static final class FlywheelMotorConstants {
            public static final int FLYWHEEL_MOTOR_ID = 3;
            public static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 4;
            // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.124133%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4800%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=1&useCustomShooterMoi=1
            public static final Slot0Configs FLYWHEEL_GAINS = new Slot0Configs()
                .withKP(0.02)
                .withKD(0.0)
                .withKS(.23092)
                .withKV(0.12149)
                .withKA(0.0082697);

            public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120);

            public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
            public static final MotorOutputConfigs FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

            public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withVelocityFilterTimeConstant(Seconds.of(0.0));
        }



        public static final int HOOD_SERVO_CHANNEL = 8;
        public static final int HOOD_SERVO_FOLLOWER_CHANNEL = 9;

        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM = new Transform3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Inches.of(21.875),
            new Rotation3d()
        );
    }

}
