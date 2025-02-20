package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(10, MotorType.kBrushless);

    SparkMaxConfig configL = new SparkMaxConfig();
    SparkMaxConfig configR = new SparkMaxConfig();

    public void ElevatorSubsystem() {
        // Restore factory defaults for both motors
        
        configL.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        configR.inverted(true).idleMode(IdleMode.kBrake).follow(leftMotor).smartCurrentLimit(40);

        configL.softLimit
        .forwardSoftLimit(48)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    
        leftMotor.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // leftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();


        // Get the encoder for position tracking
        leftMotor.getEncoder().setPosition(0);

        // Reset the encoder at startup
        resetEncoder();

    }

    // Move elevator up
    public void moveUp() {
        leftMotor.set(0.2);  // Adjust speed as needed
    }

    // Move elevator down
    public void moveDown() {
        leftMotor.set(-0.2);  // Adjust speed as needed
    }

    // Stop the elevator
    public void stopElevator() {
        leftMotor.set(0);
    }

    // Reset encoder position
    public void resetEncoder() {
        leftMotor.getEncoder().setPosition(0);
    }

    // Get current elevator position
    public double getElevatorPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // This method runs every scheduler cycle (20 ms)
    }
}
