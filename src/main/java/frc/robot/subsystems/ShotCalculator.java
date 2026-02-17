// c/o team 868
package frc.robot.subsystems;

import static frc.robot.Constants.ShotCalculationConstants.BALL_TRANSFORM_CENTER;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShotCalculationConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShotCalculator extends SubsystemBase {

    private final Drive drive;

    @AutoLogOutput
    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;
    @AutoLogOutput
    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    @AutoLogOutput
    private Pose3d targetLocation = Hub.CENTER;

    @AutoLogOutput
    private double targetDistance = 0.0;

    @AutoLogOutput
    private double targetSpeedRps = 8;

    public ShotCalculator(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drive.getPose();

        targetDistance = drivetrainPose.getTranslation()
            .getDistance(targetLocation.toPose2d().getTranslation());
        targetSpeedRps = ShotCalculationConstants.distanceToShotSpeed.get(targetDistance);

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(BALL_TRANSFORM_CENTER);

        ChassisSpeeds drivetrainSpeeds = drive.getChassisSpeeds();
        ChassisAccelerations drivetrainAccelerations = drive.getChassisAccelerations();

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose,
            targetLocation,
            drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
            5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }
}
