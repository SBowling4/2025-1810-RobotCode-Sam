package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.constants.RobotConstants;
import frc.robot.util.constants.RobotConstants.ArmConstants;

public class Config {
    
    public static TalonFXConfiguration getPitchConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimit = RobotConstants.GLOBAL_CURRENT_LIMIT_BASE;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Feedback.SensorToMechanismRatio = 15; 
    
        return cfg;
    }
    
    public static TalonFXConfiguration getExtenderConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimit = RobotConstants.GLOBAL_CURRENT_LIMIT_BASE;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Feedback.SensorToMechanismRatio = 4; 
    
        return cfg;
    }
    
    public static SparkMaxConfig getRollConfig() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.smartCurrentLimit(40);
        cfg.idleMode(SparkMaxConfig.IdleMode.kBrake);

    
        return cfg;
    }
    
    public static SparkMaxConfig getArmConfig1() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.smartCurrentLimit(RobotConstants.GLOBAL_CURRENT_LIMIT_BASE);
        cfg.idleMode(SparkMaxConfig.IdleMode.kBrake);
    
        return cfg;
    }
    
    public static SparkMaxConfig getArmConfig2() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.smartCurrentLimit(RobotConstants.GLOBAL_CURRENT_LIMIT_BASE);
        cfg.idleMode(SparkMaxConfig.IdleMode.kBrake);

        cfg.follow(ArmConstants.MOTOR_ID_1);
    
        return cfg;
    }
    
    public static SparkMaxConfig getIntakeConfig() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.smartCurrentLimit(RobotConstants.GLOBAL_CURRENT_LIMIT_BASE);
        cfg.idleMode(SparkMaxConfig.IdleMode.kCoast);
    
        return cfg;
    }
}
