// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO{

    private final SparkMax m_frontIntake = new SparkMax(Constants.FRONT_INTAKE_MOTOR, MotorType.kBrushless);
    private final DigitalInput beamBreak = new DigitalInput(2);

    private Alert intakeMotorDisconnectAlert;

      public IntakeIOReal() {

        SparkBaseConfig intakeConfig = new SparkBaseConfig() {};
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.smartCurrentLimit(35);
        intakeConfig.voltageCompensation(12.0);
        intakeConfig.inverted(true);

        m_frontIntake.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        intakeMotorDisconnectAlert = new Alert("Intake motor is not present on CAN", AlertType.kError);

    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = m_frontIntake.getAppliedOutput();
        inputs.intakeCurrentAmps =  m_frontIntake.getOutputCurrent();
        inputs.noteInIntake = !beamBreak.get();

        Leds.getInstance().noteInIntake = !beamBreak.get();
        intakeMotorDisconnectAlert.set(m_frontIntake.hasActiveFault());

    };

    public void setIntakeSpeed(double velocity) {
        m_frontIntake.set(velocity);
    };

    public void stopIntake() {
        setIntakeSpeed(0);
    };
}
