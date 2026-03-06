// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeConstants.deployMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants.intakeMotorConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    // will compile after run
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //Alert for intake motor disconnection
    private final Alert intakeMotorDisconnectedAlert;
    //Alert for deploy motor disconnection
    private final Alert deployMotorDisconnectedAlert;
    //triggers: prevent subsystems from knowing each other to reduce dependency
    public final Trigger isDeployed = new Trigger(
        //trigger that is true when the intake is fully deployed
        () -> Math.abs(inputs.deployMotorPositionDegrees
            - IntakeConstants.DEPLOYED_ANGLE.in(Degrees))
            <= IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));

    public final Trigger isRetracted = new Trigger(
        //trigger that is true when the intake is fully retracted
        () -> Math.abs(inputs.deployMotorPositionDegrees
            - IntakeConstants.RETRACTED_ANGLE.in(Degrees))
            <= IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));

      public final Trigger isDeployMotorDisconnected = new Trigger(
          () -> !inputs.deployMotorConnected);

      public final Trigger isIntakeMotorDisconnected = new Trigger(
          () -> !inputs.intakeMotorConnected);

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.io = intakeIO;
    intakeMotorDisconnectedAlert = 
       new Alert("Intake motor disconnected.", Alert.AlertType.kError); 
    //critical alert for motor disconnection
    deployMotorDisconnectedAlert = 
       new Alert("Deploy motor disconnected.", Alert.AlertType.kError); 
    //critical alert for motor disconnection
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, every 20ms by default
    //update the inputs from the IO and log them
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  
    //check for intake motor disconnection and display if true
    intakeMotorDisconnectedAlert.set(!inputs.intakeMotorConnected);
    deployMotorDisconnectedAlert.set(!inputs.deployMotorConnected);

    //end of periodic
  }
  
  //command to deploy the intake
  public Command deploy() {
    //run the deploy motor at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setDeployVoltage(deployMotorConstants.deployVoltage), //deploy at the specified voltage
        () -> io.setDeployVoltage(0.0) //stop the deploy motor when the command ends
    ).until(() -> Units.radiansToDegrees(inputs.deployMotorPositionRad) > deployMotorConstants.deployTargetPositionDegrees)
     .withTimeout(deployMotorConstants.deployTimeoutSeconds); //safety timeout
  }

  public Command retract(){
    //run the deploy motor in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setDeployVoltage(-deployMotorConstants.deployVoltage), //retract at the specified voltage
        () -> io.setDeployVoltage(0.0)
    ).until(() -> Units.radiansToDegrees(inputs.deployMotorPositionRad) < deployMotorConstants.retractTargetPositionDegrees)
     .withTimeout(deployMotorConstants.retractTimeoutSeconds); //safety timeout
  }

  //command to intake fuel
  public Command intake() {
    //run the intake at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setIntakeVoltage(intakeMotorConstants.intakeVoltage),
        () -> io.setIntakeVoltage(0.0)
    );
  }

  public Command eject() {
    //run the intake in reverse at the specified voltage when this command is scheduled, and stop it when the command ends
    return runEnd(
        () -> io.setIntakeVoltage(-intakeMotorConstants.intakeVoltage), //eject at the specified voltage
        () -> io.setIntakeVoltage(0.0)
    );
  }

  public void stopIntake() {
    io.stopIntake();
  }

  public void stopDeploy() {
    io.stopDeploy();
  }


//end of class
}
