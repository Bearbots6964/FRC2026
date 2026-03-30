package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeConsts;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOAutoLogged inputs = new IntakeIOAutoLogged();

    private intakeState statusGoal = intakeState.OFF;

    private final LoggedTunableNumber intakekP = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kP);
    private final LoggedTunableNumber intakekD = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kD);
    private final LoggedTunableNumber intakekS = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kS);
    private final LoggedTunableNumber intakekV = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kV);

    private final LoggedTunableNumber deploykP = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kP);
    private final LoggedTunableNumber deploykD = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kD);
    private final LoggedTunableNumber deploykS = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kS);
    private final LoggedTunableNumber deploykV = new LoggedTunableNumber(
            "Intake/intake/kP", IntakeConsts.INTAKE_CONFIGS.kV);
    
    


    public enum intakeState {
        ON,
        OFF,
        TUNING
    }
}
