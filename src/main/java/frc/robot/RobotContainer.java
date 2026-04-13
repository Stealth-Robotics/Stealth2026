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
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final RobotSystem robot;

    private final Autos autos;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // private String lastAutoName = "";

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

        autoChooser.setDefaultOption("Nothing", new InstantCommand());
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
            () -> operatorController.b().getAsBoolean() || driverController.b().getAsBoolean()
        );

        Trigger shootTrigger = new Trigger(driverController.rightBumper().or(operatorController.rightBumper()));
        shootTrigger.whileTrue(robot.shoot());

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.leftBumper().whileTrue(robot.activatePrecisionDriving());
        driverController.start().onTrue(robot.forceResetOdometry());

        operatorController.leftBumper().whileTrue(robot.clearTransfer());
 
        operatorController.povUp()
            .onTrue(new InstantCommand(() -> robot.changeRPMOffset(25)));
        operatorController.povDown()
            .onTrue(new InstantCommand(() -> robot.changeRPMOffset(-25)));
    }

    /*
     * Add all our working autonomous routines to the chooser for selection on Elastic
     */
    private void addAutosToChooser() {
        autoChooser.addOption("Debug", autos.debugAuto());
    } 

    /*
     * Called repeatedly while the robot is disabled. 
     * Used to preload the selected auto routine's trajectory file to prevent lag at the start of the match.
     */
    public void disabledPeriodic() {
        // String selectedName = autoChooser.getSelected().getName();
        // if (selectedName != null && !selectedName.equals(lastAutoName)) {
        //     lastAutoName = selectedName;
        //     autos.preloadAuto(selectedName);
        // }
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();
    }

    public void toggleDisabledLeds(boolean disable) {
        robot.toggleDisabledLeds(disable);
    }

    public void setLEDBrightness(double value) {
        robot.setLEDBrightness(value);
    }

    public void resetFuelCounter() {
        robot.resetFuelShotCount();
    }
}