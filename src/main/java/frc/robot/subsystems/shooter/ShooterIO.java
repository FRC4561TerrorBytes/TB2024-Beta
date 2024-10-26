/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

/** Add your docs here. */
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public Measure<VoltageUnit> shooterVoltage = Volts.of(0.0);
        public Measure<AngularVelocityUnit> shooterVelocityMPS = RotationsPerSecond.of(0.0);
        public Measure<CurrentUnit> shooterCurrentAmps = Amp.of(0.0);
        public Measure<AngleUnit> motorPosition = Rotations.of(0.0);
        public Measure<AngularVelocityUnit> motorSetpoint = RotationsPerSecond.of(0.0);
    }
    
    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void updateInputs(ShooterIOInputs inputs) {};

    public default void setFlywheelSpeed(double velocity) {};

    public default void stopFlywheel() {};

    public default boolean getDisconnect() {return false;};
}
