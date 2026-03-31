package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeConsts;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private intakeState statusGoal = intakeState.OFF;

  private final LoggedTunableNumber intakekP =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kP);
  private final LoggedTunableNumber intakekD =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kD);
  private final LoggedTunableNumber intakekS =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kS);
  private final LoggedTunableNumber intakekV =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kV);

  private final LoggedTunableNumber deploykP =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kP);
  private final LoggedTunableNumber deploykD =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kD);
  private final LoggedTunableNumber deploykS =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kS);
  private final LoggedTunableNumber deploykV =
      new LoggedTunableNumber("Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kV);

  private final LoggedTunableNumber tuningIntakeSpeed =
      new LoggedTunableNumber("Intake/Tuning/Intake", 0.0);

  private final LoggedTunableNumber tuningDeployPosition =
      new LoggedTunableNumber("Intake/Tuning/DeployPosition", 0.0);

  private final Alert leadIntakeAlert;
  private final Alert followerIntakeAlert;
  private final Alert deployAlert;

  public Intake(IntakeIO io) {
    this.io = io;

    leadIntakeAlert = new Alert("Lead intake motor disconnected", AlertType.kError);
    followerIntakeAlert = new Alert("Follower intake motor disconnected", AlertType.kError);

    deployAlert = new Alert("Intake deploy motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    leadIntakeAlert.set(!inputs.intakeConnected);
    followerIntakeAlert.set(!inputs.intake2Connected);

    deployAlert.set(!inputs.intakeDeployConnected);
  }

  public Command setCommand(intakeState goal) {
    return runOnce(
        () -> {
          this.statusGoal = goal;
          switch (goal) {
            case ON, TUNING:
              startIntake(RotationsPerSecond.of(IntakeConstants.IntakeConsts.TARGET_RPS));
              startDeploy(Degrees.of(IntakeConstants.intakeDeploy.TARGET_POSITION_DEGREES));
              break;

            case OFF:
              stop();
              break;
            case RETRACTED:
              rectractDeploy(
                  Degrees.of(IntakeConstants.intakeDeploy.TARGET_POSITION_DEGREES_RETRACT));
              break;
            default:
              stop();
              break;
          }
        });
  }

  private void startIntake(AngularVelocity velocity) {
    io.runIntakeClosedLoop(velocity);
  }

  private void rectractDeploy(Angle posAngle) {
    io.retractIntake(posAngle);
  }

  private void startDeploy(Angle position) {
    io.deployToPosition(position);
  }

  private void stop() {
    io.stopIntake();
    io.stopDeploy();
  }

  public enum intakeState {
    ON,
    OFF,
    RETRACTED,
    TUNING
  }
}
