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
        }

        public static final int LOOKAHEAD_ITERATIONS = 2;
    }

    public static final class TalonFXConstants {

        public static final class TurnMotorConstants {
            public static final int TURN_MOTOR_ID = 10;
            public static final int TURN_ENCODER_ID = 20;

            public static final Angle MAX_TURN_ANGLE = Units.Degrees.of(180.0);
            public static final Angle MIN_TURN_ANGLE = Units.Degrees.of(-180.0);

            public static final double MOTOR_TO_TURRET_RATIO = 4.8
                * 10.0; // 4.8:1 gearing from motor to encoder, 10:1 gearing from encoder to turret
            public static final double ENCODER_TO_TURRET_RATIO = 10.0; // 10:1 gearing from encoder to turret

            public static final Slot0Configs TURN_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0);

            public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLowerLimit(30.0);

            public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        }

        public static final class FlywheelMotorConstants {
            public static final int FLYWHEEL_MOTOR_ID = 11;
            public static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 12;

            public static final Slot0Configs FLYWHEEL_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0);

            public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100);

            public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
            public static final MotorOutputConfigs FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

            public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withVelocityFilterTimeConstant(Seconds.of(0.01));
        }



        public static final int HOOD_SERVO_CHANNEL = 0;
        public static final int HOOD_SERVO_FOLLOWER_CHANNEL = 1;

        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM = new Transform3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Inches.of(21.875),
            new Rotation3d()
        );
    }

}
