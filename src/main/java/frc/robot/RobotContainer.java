// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser;

    private boolean driveFieldCentric = true;
    
    public RobotContainer() {
        robot = new RobotSystem(driverController, operatorController);

        //Add the auto chooser to our dashboard
        autos = robot.getAutos();

        autoChooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //TODO: Commented out because we don't have the driver cam hooked up yet
        //Stream the driver camera to Elastic
        // UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setResolution(640, 480);
        // camera.setFPS(30);

        //Allows us to bypass the shift tracker for testing/emergency situations
        SmartDashboard.putBoolean("Override ShiftTracker", false);

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
            () -> driverController.getRightX(),
            () -> driveFieldCentric
        );

        robot.setIntakeDefaultCommand(
            () -> driverController.getLeftTriggerAxis() > 0.01
                ? -driverController.getLeftTriggerAxis() : driverController.getRightTriggerAxis(), 
            () -> driverController.getRightTriggerAxis() > 0.01
        );

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.povDown().onTrue(new InstantCommand(() -> driveFieldCentric = !driveFieldCentric));

        driverController.rightBumper().whileTrue(robot.shoot());
        driverController.leftBumper().whileTrue(robot.clearTransfer());

        driverController.a().whileTrue(robot.rotateRobotToShoot());
        // driverController.y().whileTrue(robot.driveToClimb());
    }

    /*
     * Add all our working autonomous routines to the chooser for selecting on Elastic
     */
    private void addAutosToChooser() {
        autoChooser.addRoutine("OneCycleWin", () -> autos.oneCycle());
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
        DogLog.log("Driving Mode", driveFieldCentric ? "Field Centric" : "Robot Centric");
    }
}
