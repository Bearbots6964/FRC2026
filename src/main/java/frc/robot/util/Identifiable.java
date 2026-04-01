package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface Identifiable {
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);
    public Command sysIdDynamic(SysIdRoutine.Direction direction);

}
