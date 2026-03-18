package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    private final SwerveDriveSimulation driveSimulation;

    public IntakeIOSim(SwerveDriveSimulation driveSimulation, BooleanSupplier customIntakeCondition) {
        this.driveSimulation = driveSimulation;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            Units.Inches.of(25.5),
            Units.Inches.of(5.375),
            IntakeSide.FRONT,
            3
        );
        this.intakeSimulation.setCustomIntakeCondition(gp -> customIntakeCondition.getAsBoolean());
    }

}
