// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.subsystems.LEDSubsystem.DisplayMode;


public class RobotContainer {
    private final Driver driver = Driver.MATT;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private enum Driver {
        MATT,
        MO
    }
    
    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser;

    private boolean deploy = true;
    
    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions()
            .withCaptureDs(true)
            .withLogExtras(true)
            .withCaptureConsole(true)
            .withCaptureNt(true)
        );

        DogLog.setPdh(new PowerDistribution(63, ModuleType.kRev));

        robot = new RobotSystem(driverController, operatorController);

        //Add the auto chooser to our dashboard
        autos = robot.getAutos();

        autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Hahaha stopped the annoying warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();
        addAutosToChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    private void configureBindings() {
        robot.setDriveDefaultCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX()
        );

        if (driver.equals(Driver.MATT)) {
            robot.setIntakeDefaultCommand(
                () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
                () -> driverController.y().getAsBoolean() && deploy,
                () -> driverController.y().getAsBoolean() && !deploy
            );

            driverController.y().onTrue(new InstantCommand(() -> deploy = !deploy));
        }
        else {
            robot.setIntakeDefaultCommand(
                () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
                () -> driverController.getRightTriggerAxis() > 0.01,
                () -> driverController.rightBumper().getAsBoolean()
            );
        }

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.b().onTrue(robot.agitate());

        operatorController.rightBumper().whileTrue(robot.shoot());
        operatorController.leftBumper().whileTrue(robot.clearTransfer());
        
        operatorController.a().whileTrue(robot.keepRobotLockedWithTurret());
        operatorController.b().onTrue(robot.agitate());        
        operatorController.x().whileTrue(robot.activatePrecisionDriving());
    }

    /*
     * Add all our working autonomous routines to the chooser for selecting on Elastic
     */
    private void addAutosToChooser() {
        autoChooser.addRoutine("RightOneCyclePlusOutpost", () -> autos.rightOneCyclePlusOutpost());
        autoChooser.addRoutine("LeftOneCyclePlusDepot", () -> autos.leftOneCyclePlusDepot());
        autoChooser.addRoutine("CenterPreload", () -> autos.centerPreload());
        autoChooser.addRoutine("SimpleCenterAuto", () -> autos.simpleCenterAuto());
    } 

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();

        DogLog.log("Alliance", AllianceUtility.getAlliance().name());
        DogLog.log("Match Phase", ShiftTracker.getCurrentMatchPhase());

        String timeString = String.format(
            "%d:%02d",
            (int) ShiftTracker.getTimeLeftInShift() / 60,
            (int) ShiftTracker.getTimeLeftInShift() % 60
        );
        DogLog.log("Shift Time Left", timeString);

        DogLog.log("Hub Scorable", ShiftTracker.canScore());
    }

    // Sets default light state when disabled
    public void disabledPeriodic() {
        robot.setLEDMode(DisplayMode.DISABLED);
        robot.resetFuelShotCount();
    }

    public void resetAfterAuto() {
        robot.resetAfterAuto();
    }
}