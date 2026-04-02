// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.Shooter.ShooterGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ControllerRumbleManager;
import frc.robot.util.FuelSim;
import frc.robot.util.Identifiable;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
//    private final Climber climber;
    private final Shooter shooter;
    private final Indexer indexer;

    private final Superstructure superstructure;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Bindings
    // Driver
    private final Trigger traverseBumpTrigger = driverController.a();
    private final Trigger lockWheelsTrigger = driverController.x();
    private final Trigger alignToTowerTrigger = driverController.start();
    private final Trigger resetGyroTrigger = driverController.back();
    private final Trigger killAutoTrigger = driverController.b();
    // Operator
    private final Trigger spinShooterTrigger = operatorController.leftTrigger();
    private final Trigger shootTrigger = operatorController.rightTrigger();
    private final Trigger stopIntakeTrigger = operatorController.y();
    private final Trigger deployIntakeTrigger = operatorController.b();
    private final Trigger retractIntakeTrigger = operatorController.b()
        .and(operatorController.rightBumper());
    private final Trigger reverseIntakeTrigger = operatorController.leftBumper();
    private final Trigger manualTurretControlTrigger = operatorController.a();

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final ControllerRumbleManager rumbleManager;

    public FuelSim fuelSim;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (pose) -> {
                        });
                intake = new Intake(new IntakeIOTalonFX());
//                climber = new Climber(new ClimberIOTalonFX());
                shooter = new Shooter(new ShooterIOTalonFX(), drive::getPose, drive::getChassisSpeeds);
                indexer = new Indexer(new IndexerIOTalonFX());

                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVision(camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1)
                    );
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                configureFuelSim();

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig,
                    new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive =
                    new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                intake = new Intake(new IntakeIO() {});
//                climber = new Climber(new ClimberIO() {});
                shooter = new Shooter(new ShooterIO() {}, drive::getPose, drive::getChassisSpeeds);
                indexer = new Indexer(new IndexerIO() {
                });


                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                intake = new Intake(new IntakeIO() {});
//                climber = new Climber(new ClimberIO() {});
                shooter = new Shooter(new ShooterIO() {}, drive::getPose, drive::getChassisSpeeds);

                indexer = new Indexer(new IndexerIO() {
                });
                break;
        }
        superstructure = new Superstructure(shooter, intake, indexer, drive::getPose);

        configureAutonomous();
        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive));
        addSysIdOptions("Drive", drive);
        addSysIdOptions("Shooter", shooter);
        addSysIdOptions("Intake", intake);

        // Configure the button bindings
        configureButtonBindings();

        rumbleManager = new ControllerRumbleManager(
            (c) -> driverController.setRumble(RumbleType.kBothRumble, c));
    }

    private void addSysIdOptions(String name, Identifiable subsystem) {
        autoChooser.addOption(name + " SysId (Quasistatic Forward)", subsystem.sysIdQuasistatic(
            Direction.kForward));
        autoChooser.addOption(name + " SysId (Quasistatic Reverse)", subsystem.sysIdQuasistatic(
            Direction.kReverse));
        autoChooser.addOption(name + " SysId (Dynamic Forward)", subsystem.sysIdDynamic(
            Direction.kForward));
        autoChooser.addOption(name + " SysId (Dynamic Reverse)", subsystem.sysIdDynamic(
            Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(drive, () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

//        slowMovementTrigger.whileTrue(DriveCommands.joystickDriveSlowly(drive, () -> -driverController.getLeftY(),
//            () -> -driverController.getLeftX(), () -> -driverController.getRightX()));
//        fineTurningTrigger.whileTrue(DriveCommands.joystickDriveFollowingVelocity(drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));
//        drive.nearBumpTrigger.whileTrue(
//            DriveCommands.joystickDriveOverBump(drive, () -> -driverController.getLeftY(),
//                () -> -driverController.getLeftX()));

        // Switch to X pattern when X button is pressed
        lockWheelsTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive));

        driverController.rightTrigger().or(operatorController.rightTrigger()).whileTrue(Commands.run(drive::stopWithX, drive));
        //Deploy intake when B button is pressed, 
        //negate means the command will only run if the intake isn't already deployed
        deployIntakeTrigger.onTrue(intake.setGoalCommand(IntakeGoal.DEPLOY));
        //Retract intake when B button + Right Bumper is pressed
        retractIntakeTrigger.and(intake.isRetracted.negate()).whileTrue(intake.setGoalCommand(IntakeGoal.STOW));
        //Intake fuel while Y button is held
        stopIntakeTrigger.whileTrue(intake.setGoalCommand(IntakeGoal.IDLE));
        //Eject fuel while left bumper is held
        reverseIntakeTrigger.whileTrue(intake.setGoalCommand(IntakeGoal.EJECT));

        shootTrigger.whileTrue(DriveCommands.joystickDriveAtAngle(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(),
            shooter::getCurrentTarget, () -> -operatorController.getRightX()).alongWith(
            superstructure.runGoal()).finallyDo(superstructure::idleSubsystems));
//        shootTrigger.onTrue(indexer.setGoal(IndexerGoal.ACTIVE));
//        manualTurretControlTrigger.onTrue(shooter.setGoal(ShooterGoal.TUNING).repeatedly());

//        operatorController.povUp().onTrue(intake.setGoal(IntakeGoal.TILT));
//        operatorController.povUp().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        operatorController.povRight().whileTrue(turret.sysIdQuasistatic(Direction.kReverse));
//        operatorController.povDown().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        operatorController.povLeft().whileTrue(turret.sysIdDynamic(Direction.kReverse));

        // Reset gyro / odometry
        final Runnable resetGyro =
            Constants.currentMode == Constants.Mode.SIM ? () -> drive.setPose(
                driveSimulation.getSimulatedDriveTrainPose())
                // reset odometry to actual robot pose during
                // simulation
                : () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        resetGyroTrigger.onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    private void configureAutonomous() {
        NamedCommands.registerCommand("Shoot",  superstructure.runGoal());
        NamedCommands.registerCommand("Intake", intake.setGoalCommand(IntakeGoal.DEPLOY));
    }

    private void configureFuelSim() {
        fuelSim = new FuelSim();
        fuelSim.spawnStartingFuel();

        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
    }

    private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
        // TODO: implement method to link fuel sim with simulated intake
    }
    public void a() {
        shooter.recalc();
    }

    /**
     * Starts the controller rumble manager to provide rumble feedback during the match. Should be
     * called at beginning of teleop.
     */
    public void startControllerRumbleManager() {
        if (rumbleManager.isInitialized()) {
            // already started
            return;
        }
        CommandScheduler.getInstance().schedule(rumbleManager.fullMatchSequence());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition",
            driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
