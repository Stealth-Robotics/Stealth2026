// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Autos;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final RobotSystem robot;

    private final Autos autos;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions()
            .withNtPublish(true)
            .withCaptureDs(false)
            .withLogExtras(false)
            .withCaptureConsole(true)
        );

        robot = new RobotSystem(driverController, operatorController);

        //Lower the RoboRio's brownout voltage threshold
        RoboRioDataJNI.setBrownoutVoltage(6.3);

        //Add the auto chooser to our dashboard
        autos = robot.getAutos();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Hood encoder resetting
        SmartDashboard.putData("Hood Reset", robot.dashboardHoodReset().ignoringDisable(true));

        //Hahaha stopped the annoying warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();
        addAutosToChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        robot.setDriveDefaultCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        );

        robot.configureIntake(
            () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
            () -> driverController.getRightTriggerAxis() > 0.1,
            () -> driverController.y().getAsBoolean(),
            () -> operatorController.a().getAsBoolean() || driverController.a().getAsBoolean(),
            () -> operatorController.b().getAsBoolean() || driverController.b().getAsBoolean()
        );

        Trigger shootTrigger = new Trigger(driverController.rightBumper().or(operatorController.rightBumper()));
        shootTrigger.whileTrue(robot.shoot());

        driverController.start().onTrue(robot.forceResetOdometry());
        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.leftBumper().whileTrue(robot.activatePrecisionDriving());

        // uncomment for pigeon yaw testings
        // driverController.start().onTrue(new InstantCommand(() -> robot.getDrive().getPigeon2().reset()));

        operatorController.leftBumper().whileTrue(robot.clearTransfer());
 
        operatorController.povUp()
            .onTrue(new InstantCommand(() -> robot.changeRPMOffset(25)));
        operatorController.povDown()
            .onTrue(new InstantCommand(() -> robot.changeRPMOffset(-25)));
    }

    /*
     * Add all of our autonomous routines to the chooser for selection on Elastic
     */
    private void addAutosToChooser() {
        var autoLibrary = autos.getAutoLibrary();
        for (var auto : autoLibrary.entrySet()) {
            autoChooser.addOption(auto.getKey().name(), auto.getValue());
        }
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();
    }

    public void resetAfterAuto() {
        robot.resetAfterAuto();
    }

    public void resetFuelCounter() {
        robot.resetFuelShotCount();
    }
}