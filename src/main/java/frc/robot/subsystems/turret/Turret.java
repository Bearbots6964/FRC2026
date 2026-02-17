package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.ROBOT_TO_TURRET_TRANSFORM;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.FlywheelMotorConstants;
import frc.robot.subsystems.turret.TurretConstants.TalonFXConstants.TurnMotorConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Hub.topCenterPoint
        : Hub.oppTopCenterPoint;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.OFF;

    private final TurretVisualizer visualizer;

    private final Alert turnMotorDisconnected;
    private final Alert flywheelMotorDisconnected;
    private final Alert flywheelTurnMotorDisconnected;
    
    private final LoggedTunableNumber turnKP = new LoggedTunableNumber("Turret/Turn/kP", TurnMotorConstants.TURN_GAINS.kP);
    private final LoggedTunableNumber turnKD = new LoggedTunableNumber("Turret/Turn/kD", TurnMotorConstants.TURN_GAINS.kD);
    private final LoggedTunableNumber turnKS = new LoggedTunableNumber("Turret/Turn/kS", TurnMotorConstants.TURN_GAINS.kS);
    private final LoggedTunableNumber turnKV = new LoggedTunableNumber("Turret/Turn/kV", TurnMotorConstants.TURN_GAINS.kV);

    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP", FlywheelMotorConstants.FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD", FlywheelMotorConstants.FLYWHEEL_GAINS.kD);
    private final LoggedTunableNumber flywheelKS = new LoggedTunableNumber("Turret/Flywheel/kS", FlywheelMotorConstants.FLYWHEEL_GAINS.kS);
    private final LoggedTunableNumber flywheelKV = new LoggedTunableNumber("Turret/Flywheel/kV", FlywheelMotorConstants.FLYWHEEL_GAINS.kV);

    private final LoggedTunableNumber tuningFlywheelSpeed = new LoggedTunableNumber("Turret/Tuning/FlywheelSpeedRPS", 0.0);
    private final LoggedTunableNumber tuningHoodPosition = new LoggedTunableNumber("Turret/Tuning/HoodPosition", 0.0);


    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
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
        flywheelMotorDisconnected = new Alert("Turret flywheel motor disconnected.", AlertType.kError);
        flywheelTurnMotorDisconnected = new Alert("Turret flywheel turn motor disconnected.", AlertType.kError);
    }



    public enum TurretGoal {
        SCORING,
        PASSING,
        IDLE,
        TUNING,
        OFF
    }
}
