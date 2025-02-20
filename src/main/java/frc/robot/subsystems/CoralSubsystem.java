package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor;

    // Constructor
    public CoralSubsystem(int motorCANID, int controllerPort) {
        motor = new SparkMax(motorCANID, MotorType.kBrushless);
    }

    // @Override
    // public void periodic() {
    //     // Check if LB (left bumper) is pressed
    //     if (controller.getLeftBumper()) {
    //         motor.set(0.25); // Set motor to 25% speed
    //     }
    //     // Check if RB (right bumper) is pressed
    //     else if (controller.getRightBumper()) {
    //         motor.set(0.0); // Stop motor
    //     }
    //     // Default case: ensure motor is stopped if no bumpers are pressed
    //     else {
    //         motor.set(0.0); // Stop motor
    //     }
    // }

    // Stop the motor (helper function)
    public void stopMotor() {
        motor.set(0.0);
    }

    // Run the motor at a specified speed
    public void runMotor(double speed) {
        motor.set(speed);
    }
}
