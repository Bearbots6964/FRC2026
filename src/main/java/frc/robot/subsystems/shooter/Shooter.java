package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged logged = new ShooterIOInputsAutoLogged();

  private StatusGoal state = StatusGoal.OFF;

  private final LoggedTunableNumber shooterkP =
      new LoggedTunableNumber(
          "Shooter/shooter/kP", Constants.mainShooterConstants.SHOOTER_GAINS.kP);
  private final LoggedTunableNumber shooterkD =
      new LoggedTunableNumber(
          "Shooter/shooter/kP", Constants.mainShooterConstants.SHOOTER_GAINS.kD);
  private final LoggedTunableNumber shooterkS =
      new LoggedTunableNumber(
          "Shooter/shooter/kP", Constants.mainShooterConstants.SHOOTER_GAINS.kS);
  private final LoggedTunableNumber shooterkV =
      new LoggedTunableNumber(
          "Shooter/shooter/kP", Constants.mainShooterConstants.SHOOTER_GAINS.kV);

  private final LoggedTunableNumber hoodkP =
      new LoggedTunableNumber("Shooter/hood/kP", Constants.hoodConstants.HOOD_GAINS.kP);
  private final LoggedTunableNumber hoodkD =
      new LoggedTunableNumber("Shooter/hood/kP", Constants.hoodConstants.HOOD_GAINS.kD);
  private final LoggedTunableNumber hoodkS =
      new LoggedTunableNumber("Shooter/hood/kP", Constants.hoodConstants.HOOD_GAINS.kS);
  private final LoggedTunableNumber hoodkV =
      new LoggedTunableNumber("Shooter/hood/kP", Constants.hoodConstants.HOOD_GAINS.kV);

  private final LoggedTunableNumber tuningShooterSpeed =
      new LoggedTunableNumber("Shooter/Tuning/ShooterSpeedRPS", 0.0);
  private final LoggedTunableNumber tuningHoodPosition =
      new LoggedTunableNumber("Shooter/Tuning/HoodPosition", 0.0);

  private final Alert shooterAlert;
  private final Alert follower1Alert;
  private final Alert follower2Alert;
  private final Alert follower3Alert;

  private final Alert hoodAlert;

  public Shooter(ShooterIO io) {
    this.io = io;

    shooterAlert = new Alert("Lead shooter motor disconnected", AlertType.kError);
    follower1Alert = new Alert("Follower1 shooter motor disconnected", AlertType.kError);
    follower2Alert = new Alert("Follower2 shooter motor disconnected", AlertType.kError);
    follower3Alert = new Alert("Follower3 shooter motor disconnected", AlertType.kError);

    hoodAlert = new Alert("Hood motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(logged);

    Logger.processInputs("Shooter", logged);

    shooterAlert.set(!logged.leadMotorConnected);
    follower1Alert.set(!logged.followerMotor1Connected);
    follower2Alert.set(!logged.followerMotor2Connected);
    follower3Alert.set(!logged.followerMotor3Connected);

    hoodAlert.set(!logged.hoodMotorConnected);
  }

  public Command setState(StatusGoal stateGoal) {
    return runOnce(
        () -> {
          this.state = stateGoal;
          switch (stateGoal) {
            case ON, TUNING:
              break;
            case OFF:
              stop();
              break;
            default:
              stop();
              break;
          }
        });
  }

  public void stop() {
    io.stopShooter();
    io.stopHood();
  }

  public void setShooterSpeed(AngularVelocity speed) {
    // TODO: parker codes this stuff
  }

  public void setHoodPosition(Angle pos) {
    // TODO: parker codes thsi stuff
  }

  public enum StatusGoal {
    ON,
    OFF,
    TUNING
  }
}
