// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.Leds;

/** Add your docs here. */
public class IndexerIOReal implements IndexerIO {

    private final SparkMax m_indexer = new SparkMax(Constants.INDEXER, MotorType.kBrushless);
    private final DigitalInput m_rightLimitSwitch = new DigitalInput(1);
    private final DigitalInput m_leftLimitSwitch = new DigitalInput(3);

    private Alert indexerMotorDisconnectAlert;

    public IndexerIOReal(){


        SparkBaseConfig indexerConfig = new SparkBaseConfig() {};
        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.smartCurrentLimit(60, 25);
        m_indexer.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        indexerMotorDisconnectAlert = new Alert("Indexer motor is not present on CAN", AlertType.kError);
    }
    
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerAppliedVolts = m_indexer.getAppliedOutput();
        inputs.indexerState = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
        inputs.indexerCurrentAmps = m_indexer.getOutputCurrent();

        Leds.getInstance().noteInIndexer = !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
        Logger.recordOutput("LeftLimit",!m_leftLimitSwitch.get());
        Logger.recordOutput("RightLimit",!m_rightLimitSwitch.get());

        indexerMotorDisconnectAlert.set(m_indexer.hasActiveFault());
    }

    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    public void stopIndexer(){
        setIndexerSpeed(0);
    }

    public boolean getIndexerState(){
        return !m_rightLimitSwitch.get() || !m_leftLimitSwitch.get();
    }
}
