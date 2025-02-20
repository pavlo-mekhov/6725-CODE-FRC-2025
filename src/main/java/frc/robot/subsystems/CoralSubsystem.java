package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final XboxController controller;

    // Constructor
    public CoralSubsystem(int motorCANID, int controllerPort) {
        motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
        controller = new XboxController(controllerPort); // Initialize controller
    }

    @Override
    public void periodic() {
        // Check if LB (left bumper) is pressed
        if (controller.getLeftBumper()) {
            motor.set(0.25); // Set motor to 25% speed
        }
        // Check if RB (right bumper) is pressed
        else if (controller.getRightBumper()) {
            motor.set(0.0); // Stop motor
        }
        // Default case: ensure motor is stopped if no bumpers are pressed
        else {
            motor.set(0.0); // Stop motor
        }
    }

    // Stop the motor (helper function)
    public void stopMotor() {
        motor.set(0.0);
    }

    // Run the motor at a specified speed
    public void runMotor(double speed) {
        motor.set(speed);
    }
}
