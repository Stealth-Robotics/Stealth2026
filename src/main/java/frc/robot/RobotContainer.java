// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    /* Reset robot's encoders and states */
    public Command resetRobot() {
        return null;
    }

    /*
     * Home all the robot's subsystems to the starting configuration. We should make
     * sure the subsystems don't clash while resetting their states.
     */
    public Command homeRobot() {
        return null;
    }
}
