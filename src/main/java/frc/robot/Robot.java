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
import frc.robot.util.ShotCalculator;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private final String FRONT_LL = "limelight-front";
    private final String RIGHT_LL = "limelight-right";

    private final int LIMELIGHT_DISABLED_THROTTLE = 100;

    public Robot() {
        m_robotContainer = new RobotContainer();

        //For Elastic save loading
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        //Stop hoot replay logging
        SignalLogger.enableAutoLogging(false);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //Run the robot container's periodic
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.toggleDisabledLeds(true);

        LimelightHelpers.SetThrottle(FRONT_LL, LIMELIGHT_DISABLED_THROTTLE);
        LimelightHelpers.SetThrottle(RIGHT_LL, LIMELIGHT_DISABLED_THROTTLE);

        LimelightHelpers.SetIMUMode(FRONT_LL, 0);
        LimelightHelpers.SetIMUMode(RIGHT_LL, 0);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        m_robotContainer.toggleDisabledLeds(false);
        m_robotContainer.resetFuelCounter();

        LimelightHelpers.SetThrottle(FRONT_LL, 0);
        LimelightHelpers.SetThrottle(RIGHT_LL, 0);

        // LimelightHelpers.SetIMUMode(FRONT_LL, 4);
        // LimelightHelpers.SetIMUMode(RIGHT_LL, 4);

        //Reset the ShotCalculator's velocity filters 
        ShotCalculator.resetFilters();
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
        m_robotContainer.resetAfterAuto();
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
