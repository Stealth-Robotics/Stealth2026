// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ShiftTracker;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        //For Elastic save loading
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        //Stop hoot replay logging
        SignalLogger.enableAutoLogging(false);

        // Mode 0 - EXTERNAL_ONLY: Uses only external robot IMU data (e.g., Pigeon 2) via SetRobotOrientation(). No internal IMU processing.
        // Mode 1 - EXTERNAL_SEED: Uses external IMU data for botpose, but constantly seeds the internal IMU offset to match the external source, preparing for a switch to internal modes.
        // Mode 2 - INTERNAL_ONLY: Relies solely on the internal fused IMU yaw.
        // Mode 3 - INTERNAL_MT1_ASSIST: Fuses the internal IMU with MegaTag1 (MT1) vision yaw estimates to slowly correct IMU drift.
        // Mode 4 - INTERNAL_EXTERNAL_ASSIST (Recommended): Fuses the internal IMU with external IMU data using a complementary filter, offering 1kHz motion updates while eliminating drift through external correction. 

        LimelightHelpers.SetIMUMode("limelight-robot", 0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //Run the robot container's periodic
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
    
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {        
        ShiftTracker.start();
        
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (!ShiftTracker.isRunning()) {
            ShiftTracker.start();
        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        ShiftTracker.reset();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
