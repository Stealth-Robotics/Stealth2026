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

    private boolean isDisabled = true;

    public Robot() {
        m_robotContainer = new RobotContainer();

        //For Elastic save loading
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        //Stop hoot replay logging
        SignalLogger.enableAutoLogging(false);

        //Set the limelight's tag filter & IMU alpha
        for (String ll : LimelightConstants.LIMELIGHTS) {
            // LimelightHelpers.SetFiducialIDFiltersOverride(ll, LimelightConstants.TAG_FILTER_MODE.getTags());
            LimelightHelpers.SetIMUAssistAlpha(ll, LimelightConstants.IMU_ALPHA);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //Run the robot container's periodic
        m_robotContainer.periodic();

        //Limelight IMU
        if (isDisabled) {
            for (String ll : LimelightConstants.LIMELIGHTS) {
                LimelightHelpers.SetThrottle(ll, LimelightConstants.LIMELIGHT_DISABLED_THROTTLE);
                LimelightHelpers.SetIMUMode(ll, LimelightConstants.DISABLED_IMU_MODE);
            }
        }
        else {
            for (String ll : LimelightConstants.LIMELIGHTS) {
                LimelightHelpers.SetThrottle(ll, 0);
                LimelightHelpers.SetIMUMode(ll, LimelightConstants.ENABLED_IMU_MODE);
            }
        }
    }

    @Override
    public void disabledInit() {
        isDisabled = true;

        m_robotContainer.toggleDisabledLeds(true);
        m_robotContainer.setLEDBrightness(0.05);

        Elastic.selectTab("Disabled");
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
        isDisabled = false;

        m_robotContainer.toggleDisabledLeds(false);
        m_robotContainer.setLEDBrightness(0.5);

        m_robotContainer.resetFuelCounter();

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
