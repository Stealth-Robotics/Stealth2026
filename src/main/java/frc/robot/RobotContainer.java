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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.Autos.AutoPosition;
import frc.robot.subsystems.LEDSubsystem.DisplayMode;


public class RobotContainer {
    private final Driver driver = Driver.MO;

    private enum Driver {
        MATT,
        MO
    }

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser;

    private boolean deployOverRetract = true;
    
    public RobotContainer() {
        DogLog.setEnabled(true);

        DogLog.setOptions(new DogLogOptions()
            .withCaptureDs(true)
            .withLogExtras(false)
            .withCaptureConsole(true)
            .withLogEntryQueueCapacity(1500) //Raise the maximum number of logs that can be queued
        );

        //DogLog.setPdh(new PowerDistribution(63, ModuleType.kRev));

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
                () -> driverController.y().getAsBoolean() && deployOverRetract,
                () -> driverController.y().getAsBoolean() && !deployOverRetract
            );

            driverController.y().onTrue(new InstantCommand(() -> deployOverRetract = !deployOverRetract));
        }
        else {
            robot.setIntakeDefaultCommand(
                () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
                () -> driverController.getRightTriggerAxis() > 0.01,
                () -> driverController.rightBumper().getAsBoolean()
            );
        }

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.leftBumper().whileTrue(robot.activatePrecisionDriving());
        
        //Operator Controls

        operatorController.rightBumper().whileTrue(robot.shoot());
        operatorController.leftBumper().whileTrue(robot.clearTransfer());
        
        operatorController.b().onTrue(robot.agitate());

        operatorController.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> robot.changeRPMOffset(2))));
        operatorController.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> robot.changeRPMOffset(-2))));
    }

    /*
     * Add all our working autonomous routines to the chooser for selection on Elastic
     */
    private void addAutosToChooser() {
        //Newgen autos
        autoChooser.addRoutine("LeftTrench2Cycle", () -> autos.trench2Cycle(AutoPosition.LEFT));
        autoChooser.addRoutine("RightTrench2Cycle", () -> autos.trench2Cycle(AutoPosition.RIGHT));

        autoChooser.addRoutine("LeftTrenchBump", () -> autos.trenchBump(AutoPosition.LEFT));
        autoChooser.addRoutine("RightTrenchBump", () -> autos.trenchBump(AutoPosition.RIGHT));

        autoChooser.addRoutine("LeftDoubleBump", () -> autos.doubleBump(AutoPosition.LEFT));
        autoChooser.addRoutine("RightDoubleBump", () -> autos.doubleBump(AutoPosition.RIGHT));
        
        //OG autos
        autoChooser.addRoutine("Right1CyclePlusOutpost", () -> autos.right1CyclePlusOutpost());
        autoChooser.addRoutine("Left1CyclePlusDepot", () -> autos.left1CyclePlusDepot());
    } 

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();

        DogLog.log("Alliance", AllianceUtility.getAlliance().name());
    }

    // Sets default light state when disabled
    public void disabledLeds() {
        robot.setLEDMode(DisplayMode.DISABLED);
    }

    public void resetFuelCounter() {
        robot.resetFuelShotCount();
    }

    public void resetAfterAuto() {
        robot.resetAfterAuto();
    }
}