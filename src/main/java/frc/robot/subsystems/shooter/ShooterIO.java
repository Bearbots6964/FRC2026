package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public boolean leadMotorConnected = false;
    public boolean followerMotor1Connected = false;
    public boolean followerMotor2Connected = false;
    public boolean followerMotor3Connected = false;
    public boolean hoodMotorConnected = false;

    public Voltage leadMotorVoltage = Volts.zero();
    public Current leadMotorCurrent = Amps.zero();
    public AngularVelocity leadMotorVelocity = RotationsPerSecond.zero();
    public AngularAcceleration leadmotorAccel = RotationsPerSecondPerSecond.zero();

    public Voltage hoodMotorVoltage = Volts.zero();
    public Current hoodMotorCurrent = Amps.zero();
    public AngularVelocity hoodMotorAngularVelocity = RotationsPerSecond.zero();
    public Angle hoodPosition = Rotations.zero();
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void startShooter(AngularVelocity velocity) {}

  public default void setHoodPosition(Angle position) {}

  public default void stopShooter() {}

  public default void stopHood() {}

  public default void setShooterPIDCOnstants(double kP, double kD, double kV, double kS) {}

  public default void setHoodPIDCOnstants(double kP, double kD, double kV, double kS) {}
}
