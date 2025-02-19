package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CoralSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    // Constructor
    public CoralSubsystem(int motorCANID, int controllerPort) {
        motor = new CANSparkMax(motorCANID, MotorType.kBrushless);
    }

    // @Override
    // public void periodic() {
    //     // Check if LB (left bumper) is pressed
    //     if (controller.getLeftBumper()) {
    //         motor.set(0.25); // Set motor to 25% speed
    //     }
    //     // Check if RB (right bumper) is pressed
    //     else if (controller.getRightBumper()) {
    //         motor.set(0.0); // Set motor to 0% speed (stop)
    //     }
    //     // Default case: stop the motor
    //     else {
    //         motor.set(0.0);
    //     }
    // }

    // Stop the motor
    public void stopMotor() {
        motor.set(0.0);
    }
        // Stop the motor
        public void runMotor(double speed) {
            motor.set(speed);
        }
}