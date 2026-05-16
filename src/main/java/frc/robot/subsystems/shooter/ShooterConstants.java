package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;

public class ShooterConstants {

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
        public static final InterpolatingDoubleTreeMap INDEXER_MAP = new InterpolatingDoubleTreeMap();

        private static void f(double dist, double sh, double ho) {
            SHOT_MAP.put(dist, new ShotData(sh, ho));
        }
        private static void t(double dist, double t) {
            TOF_MAP.put(dist, t);
        }
        private static void i(double dist, double v) {
            INDEXER_MAP.put(dist, v);
        }

        static {
            // TODO: Fill these maps with data from testing

            f(1.5, 32, 0);
            f(2, 32, 6);
            f(2.5, 35, 9);
            f(3, 35, 12);
            f(3.3, 40, 13);
//            f(3.5, 37, 13);
            f(4, 40, 17);
            f(4.5, 44, 17);
            f(5, 50, 17);
            f(10, 85, 30);

            t(2.5, 0.94);

            i(2, 10);
            i(2.5, 11);
            i(3, 15);
            i(3.5, 19);
            i(5, 22);
            i(10, 50);



//            SHOT_MAP.put(1.9, new ShotData(49, 5));
//            SHOT_MAP.put(2.2, new ShotData(50, 9));
//            SHOT_MAP.put(3.0, new ShotData(50, 14));
//            SHOT_MAP.put(3.4, new ShotData(54, 16));
//            SHOT_MAP.put(3.6, new ShotData(54, 18.4));
//            SHOT_MAP.put(4.84, new ShotData(60, 20));
//            SHOT_MAP.put(10.0, new ShotData(60, 27));
//
//            TOF_MAP.put(5.227626, 1.5458333333);
//            TOF_MAP.put(4.25, 1.525);
//            TOF_MAP.put(3.654, 1.15);
//            TOF_MAP.put(3.269, 1.0916666667);
//            TOF_MAP.put(2.755, 0.8458333333);
        }

        public static final int LOOKAHEAD_ITERATIONS = 2;
    }

    public static final class TalonFXConstants {

        public static final class TurnMotorConstants {
            public static final int TURN_MOTOR_ID = 8;
            public static final int TURN_ENCODER_ID = 9;

            public static final Angle MAX_TURN_ANGLE = Degrees.of(180.0);
            public static final Angle MIN_TURN_ANGLE = Degrees.of(-190.0);

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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            public static final double MAX_OUTPUT_VOLTS = 7.0;
        }

        public static final class FlywheelMotorConstants {
            /*  ^ front/top
            M        F2
            ----------- -> right
            F1       F3
             */
            public static final int FLYWHEEL_MOTOR_ID = 6;
            public static final int FLYWHEEL_FOLLOWER_1_MOTOR_ID = 7;
            public static final int FLYWHEEL_FOLLOWER_2_MOTOR_ID = 8;
            public static final int FLYWHEEL_FOLLOWER_3_MOTOR_ID = 9;
            // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A0%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.124133%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A4800%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=1&useCustomShooterMoi=1
            public static final Slot0Configs FLYWHEEL_GAINS = new Slot0Configs()
                .withKP(0.15662)
                .withKD(0.0)
                .withKS(.5)
                .withKV(0.1358851675)
                .withKA(0.23979);

            public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120);

            public static final MotorOutputConfigs LEFT_FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
            public static final MotorOutputConfigs RIGHT_FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

            public static final double GEAR_RATIO = (34.0 / 30.0) * (26.0 / 28.0);
            public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withVelocityFilterTimeConstant(Seconds.of(0.0))
                .withSensorToMechanismRatio(GEAR_RATIO);

        }




        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM = new Transform3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Inches.of(21.875),
            new Rotation3d()
        );

        public static final class HoodMotorConstants {
            public static final int HOOD_MOTOR_ID = 10;
            public static final Angle MAX_HOOD_ANGLE = Degrees.of(15.0);

            public static final double MOTOR_TO_HOOD_RATIO = (50.0 / 14.0) * (360.0 / 50.0);

            public static final Slot0Configs HOOD_GAINS = new Slot0Configs()
                .withKP(90.0)
                .withKD(0.0)
                .withKS(0.339)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.589)
                .withGravityType(GravityTypeValue.Elevator_Static);

            public static final MotionMagicConfigs HOOD_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(3.0));

            public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40.0);

            public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);



        }
    }

}
