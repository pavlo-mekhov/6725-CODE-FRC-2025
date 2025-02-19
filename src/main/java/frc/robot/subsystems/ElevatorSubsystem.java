package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final CANEncoder leftEncoder;

    public ElevatorSubsystem() {
        // Restore factory defaults for both motors
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        // Set the right motor to follow the left motor
        rightMotor.follow(leftMotor);

        // Set brake mode for holding position when stopped
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // Set current limits to protect motors
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        // Get the encoder for position tracking
        leftEncoder = leftMotor.getEncoder();

        // Reset the encoder at startup
        resetEncoder();

        // Optional: Set soft limits to avoid over-travel
        leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 48); // Example upper limit
        leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);   // Example lower limit
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
        leftEncoder.setPosition(0);
    }

    // Get current elevator position
    public double getElevatorPosition() {
        return leftEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method runs every scheduler cycle (20 ms)
    }
}
