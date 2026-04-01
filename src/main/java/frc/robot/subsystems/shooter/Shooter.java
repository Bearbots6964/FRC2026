package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.FLYWHEEL_RADIUS;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.PASSING_SPOT_LEFT;
import static frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants.PASSING_SPOT_RIGHT;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TalonFXConstants.HoodMotorConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TargetingConstants;
import frc.robot.util.Identifiable;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements Identifiable {

    private final ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    public Rotation2d targetAngle = new Rotation2d();

    @Getter
    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Hub.topCenterPoint
        : Hub.oppTopCenterPoint;

    @AutoLogOutput
    private ShooterGoal goal = ShooterGoal.SCORING;

    @AutoLogOutput
    private HoodGoal hoodGoal = HoodGoal.IDLE;


    private final Alert flywheelMotorDisconnected;
    private final Alert flywheelFollowerMotor1Disconnected;
    private final Alert flywheelFollowerMotor2Disconnected;
    private final Alert flywheelFollowerMotor3Disconnected;

    private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Turret/Hood/kP",
        HoodMotorConstants.HOOD_GAINS.kP);
    private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Turret/Hood/kD",
        HoodMotorConstants.HOOD_GAINS.kD);
    private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Turret/Hood/kS",
        HoodMotorConstants.HOOD_GAINS.kS);
    private final LoggedTunableNumber hoodKV = new LoggedTunableNumber("Turret/Hood/kV",
        HoodMotorConstants.HOOD_GAINS.kV);

    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kD);
    private final LoggedTunableNumber flywheelKS = new LoggedTunableNumber("Turret/Flywheel/kS",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kS);
    private final LoggedTunableNumber flywheelKV = new LoggedTunableNumber("Turret/Flywheel/kV",
        FlywheelMotorConstants.FLYWHEEL_GAINS.kV);

    private final LoggedTunableNumber tuningFlywheelSpeed = new LoggedTunableNumber(
        "Turret/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodPosition = new LoggedTunableNumber(
        "Turret/Tuning/HoodPosition", 0.0);

    private final SysIdRoutine flywheelRoutine;
    private final SysIdRoutine hoodRoutine;

    public void recalc() {
        currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Hub.topCenterPoint
            : Hub.oppTopCenterPoint;
    }


    public Shooter(ShooterIO io, Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.robotPoseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        flywheelMotorDisconnected = new Alert("Turret flywheel motor disconnected.",
            AlertType.kError);
        flywheelFollowerMotor1Disconnected = new Alert(
            "Turret flywheel follower motor 1 disconnected.", AlertType.kError);

        flywheelFollowerMotor2Disconnected = new Alert(
            "Turret flywheel follower motor 2 disconnected.", AlertType.kError);
        flywheelFollowerMotor3Disconnected = new Alert(
            "Turret flywheel follower motor 3 disconnected.", AlertType.kError);

        SmartDashboard.putData(
            Commands.runOnce(() -> setGoal(ShooterGoal.TUNING)).withName("Turret Tuning Mode").ignoringDisable(true));

        flywheelRoutine = new SysIdRoutine(
            new Config(null, null, null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(io::setFlywheelVoltage, null, this)
        );

        hoodRoutine = new SysIdRoutine(
            new Config(Volts.per(Second).of(0.1), Volts.of(1), null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(io::setHoodVoltage, null, this)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Pose2d pose = robotPoseSupplier.get();

        if (goal == ShooterGoal.SCORING || goal == ShooterGoal.PASSING) {
            calculateShot(pose);
        }

        if (goal == ShooterGoal.PASSING) {
            setTarget(getPassingTarget(pose));
        }

        if (goal == ShooterGoal.TUNING) {
            io.setFlywheelSpeed(RotationsPerSecond.of(tuningFlywheelSpeed.get()));
            io.setHoodPosition(Degrees.of(tuningHoodPosition.get()));
        }

        Logger.recordOutput("Turret/Distance To Target",
            ShooterCalculator.getDistanceToTarget(pose, currentTarget));

        updateTunables();

        flywheelMotorDisconnected.set(!inputs.leadMotorConnected);
        flywheelFollowerMotor1Disconnected.set(!inputs.followerMotor1Connected);
        flywheelFollowerMotor2Disconnected.set(!inputs.followerMotor2Connected);
        flywheelFollowerMotor3Disconnected.set(!inputs.followerMotor3Connected);
    }

    public Command setGoal(ShooterGoal goal) {
        return runOnce(() -> {
            this.goal = goal;
            switch (goal) {
                case SCORING, TUNING:
                    setTarget(Hub.topCenterPoint);
                    break;
                case PASSING:
                    setTarget(getPassingTarget(robotPoseSupplier.get()));
                    break;
                case IDLE, OFF:
                    stop();
                    break;
            }
        });
    }

    public Command setHoodGoal(HoodGoal hoodGoal) {
        return runOnce(() -> this.hoodGoal = hoodGoal);
    }

    public Command runEndHood() {
        return runEnd(() -> this.hoodGoal = HoodGoal.ACTIVE, () -> this.hoodGoal = HoodGoal.IDLE);
    }


    public void setTarget(Translation3d target) {
        this.currentTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    private Translation3d getPassingTarget(Pose2d pose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = robotPoseSupplier.get().getMeasureY()
            .gt(Units.Meters.of(FieldConstants.fieldWidth / 2.0));

        return isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
    }

    private void calculateShot(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        ShotData calculatedShot = ShooterCalculator.iterativeMovingShotFromMap(robotPose,
            fieldSpeeds, currentTarget,
            TargetingConstants.LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle = ShooterCalculator.calculateAzimuthAngle(robotPose,
            calculatedShot.target());
        targetAngle = new Rotation2d(azimuthAngle);
        if (hoodGoal == HoodGoal.ACTIVE) io.setHoodPosition(calculatedShot.getHoodAngle());
        else io.setHoodPosition(Degrees.of(0.0));
        io.setFlywheelSpeed(
            ShooterCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
                FLYWHEEL_RADIUS));
//        io.setFlywheelSpeed(calculatedShot.getExitAngularVelocity());

        Logger.recordOutput("Turret/CalculatedShot", calculatedShot);
    }

    private void updateTunables() {
        if (hoodKP.hasChanged(hashCode())
            || hoodKD.hasChanged(hashCode())
            || hoodKV.hasChanged(hashCode())
            || hoodKS.hasChanged(hashCode())) {
            io.setHoodPIDConstants(hoodKP.get(), hoodKD.get(), hoodKV.get(), hoodKS.get());
        }

        if (flywheelKP.hasChanged(hashCode())
            || flywheelKD.hasChanged(hashCode())
            || flywheelKV.hasChanged(hashCode())
            || flywheelKS.hasChanged(hashCode())) {
            io.setFlywheelPIDConstants(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(),
                flywheelKS.get());
        }

    }


    private void stop() {
        io.stopFlywheel();
        io.stopHood();
    }

    public static double hoodAngleToPosition(Angle hoodAngle) {
        return hoodAngle.in(
            Radians); // TODO: Implement this conversion based on the specific hardware setup
    }

    public static Angle actualHoodPositionToAngle(double hoodPosition) {
        // precondition: hoodPosition is in millimeters and is between 0 and 100 mm. it represents the current stroke length of the servo
        // Point AB is the point of pivot of the hood, side A is the distance between the pivot and the point where the servo connects to the shooter assembly, and side B is the distance between the pivot and the point where the servo connects to the hood.
        // Point AC is the point where the servo connects to the shooter assembly, and point BC is the point where the servo connects to the hood.
        // The angle between A and the horizontal plane intersecting AB is 16.137492 degrees from horizontal. This is static.
        Distance sideA = Inches.of(9.888568);
        Distance sideB = Inches.of(6.25);
        Distance sideC =
            Millimeters.of(hoodPosition).plus(Millimeters.of(168.0)); // add the offset from the servo stroke to the actual length of the side

        Angle angle = Radians.of(Math.acos(
            (Math.pow(sideA.in(Inches), 2) + Math.pow(sideB.in(Inches), 2) - Math.pow(sideC.in(Inches), 2))
                / (2 * sideA.in(Inches) * sideB.in(Inches))
        ));
        return angle.minus(Degrees.of(16.1374915)).minus(Degrees.of(4.666254)); // subtract the static angle to get the actual hood angle from horizontal
    }

    public static double actualHoodAngleToPosition(Angle hoodAngle) {
        // precondition: hoodAngle is the angle between the hood and the horizontal plane, in degrees. 0 degrees means the hood is parallel to the ground, and positive angles mean the hood is angled upwards.
        // Point AB is the point of pivot of the hood, side A is the distance between the pivot and the point where the servo connects to the shooter assembly, and side B is the distance between the pivot and the point where the servo connects to the hood.
        // Point AC is the point where the servo connects to the shooter assembly, and point BC is the point where the servo connects to the hood.
        // The angle between A and the horizontal plane intersecting AB is 16.137492 degrees from horizontal. This is static.
        Distance sideA = Inches.of(9.888568);
        Distance sideB = Inches.of(6.25);
        Angle staticAngle = Degrees.of(16.1374915).plus(Degrees.of(4.666254)); // add a static angle to account for the fact that 0 degrees on our hood does not correspond to 0 mm of servo stroke

        Angle angleC = hoodAngle.plus(staticAngle);

        double sideCLength = Math.sqrt(
            Math.pow(sideA.in(Inches), 2) + Math.pow(sideB.in(Inches), 2)
                - 2 * sideA.in(Inches) * sideB.in(Inches) * Math.cos(angleC.in(Radians))
        );

        double servoStroke = sideCLength - Millimeters.of(168.0).in(Inches); // subtract the offset from the actual length of the side to get the servo stroke length

        return Inches.of(servoStroke).in(Millimeters);
    }






    public static Angle hoodPositionToAngle(double hoodPosition) {
        return Radians.of(
            hoodPosition); // TODO: Implement this conversion based on the specific hardware setup
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return flywheelRoutine.dynamic(dir);
    }


    public enum ShooterGoal {
        SCORING,
        PASSING,
        IDLE,
        TUNING,
        OFF
    }

    public enum HoodGoal {
        ACTIVE,
        IDLE
    }
}
