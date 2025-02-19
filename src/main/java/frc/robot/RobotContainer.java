// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

    // Subsystems
    private final CoralSubsystem coralSubsystem;
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    // Xbox Controller
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final XboxController controller = new XboxController(0);

    // Default drive command
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY()
    );

    // Constructor
    public RobotContainer() {
        coralSubsystem = new CoralSubsystem(21, 0);  // Example CAN ID 21, Controller Port 0

        // Set default commands for subsystems
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        coralSubsystem.setDefaultCommand(new RunCommand(coralSubsystem::periodic, coralSubsystem));

        // Elevator safety setup
        elevator.resetEncoder();

        // Configure button bindings
        configureBindings();
    }

    // Configure Xbox controller buttons for elevator control
    private void configureBindings() {
        // Gyro reset with D-pad down
        m_driverController.povDown().whileTrue(drivebase.gyroResetCommand());

        // Elevator controls
        m_driverController.y().whileTrue(new RunCommand(elevator::moveUp, elevator));
        m_driverController.a().whileTrue(new RunCommand(elevator::moveDown, elevator));
        m_driverController.x().onTrue(new RunCommand(elevator::stopElevator, elevator));
        // Coral subsystem controls
        m_driverController.leftBumper().whileTrue(new RunCommand(() -> coralSubsystem.runMotor(0.25), coralSubsystem));
        m_driverController.rightBumper().whileTrue(new RunCommand(coralSubsystem::stopMotor, coralSubsystem));
      }

    // Autonomous Command (if needed)
    public Command getAutonomousCommand() {
        return null;
    }
}