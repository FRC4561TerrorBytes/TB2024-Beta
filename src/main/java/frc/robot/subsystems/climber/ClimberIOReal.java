package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;;

/*
 * 
 */
public class ClimberIOReal implements ClimberIO {

    private final SparkMax climberMotor = new SparkMax(28, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private Alert climberDisconnect;

    public ClimberIOReal() {
        climberDisconnect = new Alert("Climber disconnected from CAN", AlertType.kError);
        
        //Configure magnet limit switch
        LimitSwitchConfig limitConfig = new LimitSwitchConfig();
        limitConfig.forwardLimitSwitchEnabled(true);
        limitConfig.forwardLimitSwitchType(Type.kNormallyOpen);

        //Climber config
        SparkBaseConfig climberConfig = new SparkBaseConfig() {};
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.smartCurrentLimit(15);
        
        climberConfig.apply(limitConfig);

        climberMotor.configure(climberConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
        encoder = climberMotor.getEncoder();
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberAppliedVolts = climberMotor.getAppliedOutput();
        inputs.climberCurrentAmps = climberMotor.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.climberLimitSwitch = climberMotor.getForwardLimitSwitch().isPressed();
        inputs.climberTempC = climberMotor.getMotorTemperature();
        climberDisconnect.set(climberMotor.hasActiveFault());
    }

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stopClimber() {
        setClimberSpeed(0);
    }
}