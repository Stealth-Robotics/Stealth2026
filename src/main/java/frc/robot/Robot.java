// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.ShiftTracker;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        /* Configure DogLog for use in a match & for testing
         * Some of these options should be modified at competitions for better performance
        */
        DogLog.setOptions(
            new DogLogOptions()
                .withCaptureDs(false)
                .withCaptureNt(false)
                .withNtPublish(false)
                .withCaptureConsole(false)
                .withNtTunables(true)
                .withLogExtras(true)
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //Run the robot container's periodic manually
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        ShiftTracker.start();
        
        m_robotContainer.homeRobot().schedule();

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

        /*
          ! Good for testing, but may want to remove for actual competition because we may not want to reset between
          ! auto and teleop
        */
        m_robotContainer.homeRobot().schedule();
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
