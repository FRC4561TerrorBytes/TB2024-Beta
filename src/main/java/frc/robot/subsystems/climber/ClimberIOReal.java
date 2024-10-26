package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberIOReal implements ClimberIO {

    private final CANSparkMax climberMotor = new CANSparkMax(28, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch limitSwitch;
    Alert climberDisconnect;

    public ClimberIOReal() {
        climberDisconnect = new Alert("Climber disconnected from CAN", AlertType.kError);
        climberMotor.restoreFactoryDefaults();
        
        encoder = climberMotor.getEncoder();
        climberMotor.setSmartCurrentLimit(15);
        climberMotor.setIdleMode(IdleMode.kBrake);

        limitSwitch = climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        limitSwitch.enableLimitSwitch(true);

        climberMotor.burnFlash();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.climberLimitSwitch = limitSwitch.isPressed();
        inputs.climberTempC = climberMotor.getMotorTemperature();
        climberDisconnect.set(climberMotor.getFaults() != 0);
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}