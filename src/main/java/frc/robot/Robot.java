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
import frc.robot.util.Elastic;
import frc.robot.util.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotCalculator;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        //For Elastic save loading
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        //Stop hoot replay logging
        SignalLogger.enableAutoLogging(false);

        //Set the limelight's tag filter
        for (String ll : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetFiducialIDFiltersOverride(ll, LimelightConstants.TAG_FILTER_MODE.getTags());
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //Run the robot container's periodic
        m_robotContainer.periodic();

        double robotHeading = m_robotContainer.getRobotYawDegrees();
        
        for (String limelight : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetRobotOrientation(limelight, robotHeading, 0, 0, 0, 0, 0);
        }
    }

    @Override
    public void disabledInit() {
        m_robotContainer.toggleDisabledLeds(true);

        //Disabled IMU mode
        for (String ll : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetThrottle(ll, LimelightConstants.LIMELIGHT_DISABLED_THROTTLE);
            LimelightHelpers.SetIMUMode(ll, LimelightConstants.DISABLED_IMU_MODE);
        }

        Elastic.selectTab("Disabled");
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
        m_robotContainer.toggleDisabledLeds(false);
        m_robotContainer.resetFuelCounter();

        for (String ll : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetIMUAssistAlpha(ll, 0.001); //0.05 old value
            LimelightHelpers.SetThrottle(ll, 0);
        }

        Elastic.selectTab("Teleoperated");

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

        //Auto IMU mode
        for (String ll : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetIMUMode(ll, LimelightConstants.AUTO_IMU_MODE);
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

        //Teleop IMU mode
        for (String ll : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetIMUMode(ll, LimelightConstants.TELEOP_IMU_MODE);
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
