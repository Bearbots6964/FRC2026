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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ControllerRumbleManager;
import java.util.prefs.PreferencesFactory;
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

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private final ControllerRumbleManager rumbleManager;

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
                this.vision = new Vision(
                    drive,
                    new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                    new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                intake = new Intake(new IntakeIOTalonFX()); //Instantiate intake

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

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
                intake = new Intake(new IntakeIO() {
                });

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    (pose) -> {
                    });
                vision = new Vision(drive, new VisionIO() {
                }, new VisionIO() {
                });
                intake = new Intake(new IntakeIO() {
                });

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();

        rumbleManager = new ControllerRumbleManager(
            (c) -> controller.setRumble(RumbleType.kBothRumble, c));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(),
            () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller.a().whileTrue(
            DriveCommands.joystickDriveAtAngle(drive, () -> -controller.getLeftY(),
                () -> -controller.getLeftX(), () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        //Deploy intake when B button is pressed
        controller.b().onTrue(intake.deployToPosition(IntakeConstants.deployVoltage, Math.toRadians(90)));
        //Retract intake when B button + Right Bumper is pressed
        controller.b().and(controller.rightBumper()).whileTrue(intake.retractToPosition(IntakeConstants.deployVoltage, 0.0));
        //Intake fuel while Y button is held
        controller.y().whileTrue(intake.intake(IntakeConstants.intakeVoltage));
        //Eject fuel while left bumper is held
        controller.leftBumper().whileTrue(intake.eject(IntakeConstants.intakeVoltage));


        // Reset gyro / odometry
        final Runnable resetGyro =
            Constants.currentMode == Constants.Mode.SIM ? () -> drive.setPose(
                driveSimulation.getSimulatedDriveTrainPose())
                // reset odometry to actual robot pose during
                // simulation
                : () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
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
