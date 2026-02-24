// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotTrajectoryCalculator;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser;

    private boolean driveFieldCentric = true;
    
    public RobotContainer() {
        robot = new RobotSystem(driverRumble(), operatorRumble());

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
        SmartDashboard.putBoolean("Force Allow Shooting", true);

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

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.povDown().onTrue(new InstantCommand(() -> driveFieldCentric = !driveFieldCentric));

        driverController.rightBumper().whileTrue(robot.shoot());
        driverController.leftBumper().whileTrue(robot.clearTransfer());
    }

    /*
     * Add all our working autonomous routines to the chooser for selecting on Elastic
     */
    private void addAutosToChooser() {
        autoChooser.addRoutine("TestAuto", () -> autos.testAuto());
    }

    private Command driverRumble() {
        return new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1));
    }

    private Command operatorRumble() {
        return new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1));
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();

        DogLog.log("Alliance", AllianceUtility.getAlliance().name());
        DogLog.log("Match Phase", ShiftTracker.getCurrentMatchPhase());
        DogLog.log("Hub Scorable", ShiftTracker.canScore());
        DogLog.log("Driving Mode", driveFieldCentric ? "Field Centric" : "Robot Centric");
    }
}
