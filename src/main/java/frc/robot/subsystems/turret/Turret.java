package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.ROBOT_TO_TURRET_TRANSFORM;
import static frc.robot.subsystems.turret.TurretConstants.TargetingConstants.FLYWHEEL_RADIUS;
import static frc.robot.subsystems.turret.TurretConstants.TargetingConstants.PASSING_SPOT_LEFT;
import static frc.robot.subsystems.turret.TurretConstants.TargetingConstants.PASSING_SPOT_RIGHT;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.TurnMotorConstants;
import frc.robot.subsystems.turret.TurretConstants.TargetingConstants;
import frc.robot.util.LoggedTunableNumber;
import java.lang.annotation.Target;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

    private final TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Hub.topCenterPoint
        : Hub.oppTopCenterPoint;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.SCORING;

    private final TurretVisualizer visualizer;

    private final Alert turnMotorDisconnected;
    private final Alert flywheelMotorDisconnected;
    private final Alert flywheelFollowerMotorDisconnected;

    private final LoggedTunableNumber turnKP = new LoggedTunableNumber("Turret/Turn/kP",
        TurnMotorConstants.TURN_GAINS.kP);
    private final LoggedTunableNumber turnKD = new LoggedTunableNumber("Turret/Turn/kD",
        TurnMotorConstants.TURN_GAINS.kD);
    private final LoggedTunableNumber turnKS = new LoggedTunableNumber("Turret/Turn/kS",
        TurnMotorConstants.TURN_GAINS.kS);
    private final LoggedTunableNumber turnKV = new LoggedTunableNumber("Turret/Turn/kV",
        TurnMotorConstants.TURN_GAINS.kV);

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

    private final SysIdRoutine routine;


    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.robotPoseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        visualizer = new TurretVisualizer(
            () -> new Pose3d(poseSupplier
                .get()
                .rotateAround(poseSupplier.get().getTranslation(),
                    new Rotation2d(inputs.turnPosition)))
                .transformBy(ROBOT_TO_TURRET_TRANSFORM),
            fieldSpeedsSupplier);

        turnMotorDisconnected = new Alert("Turret turn motor disconnected.", AlertType.kError);
        flywheelMotorDisconnected = new Alert("Turret flywheel motor disconnected.",
            AlertType.kError);
        flywheelFollowerMotorDisconnected = new Alert(
            "Turret flywheel follower motor disconnected.", AlertType.kError);

        SmartDashboard.putData(
            Commands.runOnce(() -> setGoal(TurretGoal.TUNING)).withName("Turret Tuning Mode").ignoringDisable(true));

        routine = new SysIdRoutine(
            new Config(null, null, null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(io::setTurretVoltage, null, this)
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Pose2d pose = robotPoseSupplier.get();

        if (goal == TurretGoal.SCORING || goal == TurretGoal.PASSING) {
            calculateShot(pose);
        }

        if (goal == TurretGoal.PASSING) {
            setTarget(getPassingTarget(pose));
        }

        if (goal == TurretGoal.TUNING) {
            io.setFlywheelSpeed(RotationsPerSecond.of(tuningFlywheelSpeed.get()));
            io.setHoodPosition(tuningHoodPosition.get());
            io.setTurnSetpoint(
                TurretCalculator.calculateAzimuthAngle(pose, currentTarget, inputs.turnPosition),
                RPM.zero());
        }

        Logger.recordOutput("Turret/Distance To Target",
            TurretCalculator.getDistanceToTarget(pose, currentTarget));

        visualizer.update3dPose(inputs.turnPosition, hoodPositionToAngle(inputs.hoodPosition));
        updateTunables();

        turnMotorDisconnected.set(!inputs.turnMotorConnected);
        flywheelMotorDisconnected.set(!inputs.flywheelMotorConnected);
        flywheelFollowerMotorDisconnected.set(!inputs.flywheelMotorConnected);
    }

    public Command setGoal(TurretGoal goal) {
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

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromMap(robotPose,
            fieldSpeeds, currentTarget,
            TargetingConstants.LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle = TurretCalculator.calculateAzimuthAngle(robotPose,
            calculatedShot.target(), inputs.turnPosition);
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(
            -fieldSpeeds.omegaRadiansPerSecond / 2);
        Logger.recordOutput("Turret/vel", azimuthVelocity);
//        io.setTurnSetpoint(azimuthAngle);
        io.setTurnSetpoint(azimuthAngle, azimuthVelocity);
        io.setHoodPosition(actualHoodAngleToPosition(calculatedShot.getHoodAngle()));
        io.setFlywheelSpeed(
            TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
                FLYWHEEL_RADIUS));
//        io.setFlywheelSpeed(calculatedShot.getExitAngularVelocity());

        Logger.recordOutput("Turret/CalculatedShot", calculatedShot);
    }

    private void updateTunables() {
        if (turnKP.hasChanged(hashCode())
            || turnKD.hasChanged(hashCode())
            || turnKV.hasChanged(hashCode())
            || turnKS.hasChanged(hashCode())) {
            io.setTurnPIDConstants(turnKP.get(), turnKD.get(), turnKV.get(), turnKS.get());
        }

        if (flywheelKP.hasChanged(hashCode())
            || flywheelKD.hasChanged(hashCode())
            || flywheelKV.hasChanged(hashCode())
            || flywheelKS.hasChanged(hashCode())) {
            io.setFlywheelPIDConstants(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(),
                flywheelKS.get());
        }

    }

    @Override
    public void simulationPeriodic() {
        visualizer.updateFuel(
            TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS),
            hoodPositionToAngle(inputs.hoodPosition));
    }

    private void stop() {
        io.stopFlywheel();
        io.stopHood();
        io.stopTurn();
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

    public Rotation3d getTurretRotation() {
        return new Rotation3d(Degrees.zero(), Degrees.zero(), inputs.turnPosition);
    }

    public AngularVelocity getTurretVelocity() {
        return inputs.turnVelocity;
    }





    public static Angle hoodPositionToAngle(double hoodPosition) {
        return Radians.of(
            hoodPosition); // TODO: Implement this conversion based on the specific hardware setup
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return routine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return routine.dynamic(dir);
    }


    public enum TurretGoal {
        SCORING,
        PASSING,
        IDLE,
        TUNING,
        OFF
    }
}
