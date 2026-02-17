// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.ShiftTracker;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final RobotSystem robot;
    
    public RobotContainer() {
        robot = new RobotSystem();

        configureBindings();

        // Command bound to a dashboard button
        SmartDashboard.putData("Reset Encoders", resetRobot());

        // Allow us to toggle whether we use the shift tracker in a match
        SmartDashboard.putBoolean("Hub Dependent Shooting", true);
    }

    private void configureBindings() {
        robot.setDriveDefaultCommand(
            () -> -driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    /* Reset robot's encoders and cancel running commands */
    public Command resetRobot() {
        return new SequentialCommandGroup(
        ).finallyDo(() -> CommandScheduler.getInstance().cancelAll()).ignoringDisable(true);
    }

    /*
     * Home all the robot's subsystems to the starting configuration. We should make
     * sure the subsystems don't clash while resetting their states.
     */
    public Command homeRobot() {
        return new InstantCommand();
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        ShiftTracker.update();

        DogLog.forceNt.log("Match Phase", ShiftTracker.getCurrentMatchPhase());
        DogLog.forceNt.log("Hub Scorable", ShiftTracker.canScore());
    }
}
