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
import frc.robot.util.CurrentAlliance;
import frc.robot.util.ShiftTracker;

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

        //Stream the driver camera to Elastic
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setFPS(30);

        //Allows us to toggle whether we use the shift tracker in a match
        SmartDashboard.putBoolean("Force Allow Shooting", true);

        configureBindings();
        addAutosToChooser();
    }

    /*
     * Home all the robot's subsystems to the starting configuration. We should make
     * sure the subsystems don't clash while resetting their states.
     */
    public Command homeRobot() {
        return new InstantCommand();
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    private void configureBindings() {
        //Drive Control
        robot.setDriveDefaultCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            () -> driverController.getRightX(),
            () -> driveFieldCentric
        );

        driverController.rightStick().onTrue(robot.resetRobotHeading());
        driverController.povDown().onTrue(new InstantCommand(() -> driveFieldCentric = !driveFieldCentric));

        //Shooting Control
        driverController.rightBumper()
            .onTrue(robot.shoot())
            .onFalse(new InstantCommand(() -> robot.shoot().cancel()));
    }

    /*
     * Add all our working autonomous routines to the chooser for selecting on Elastic
     */
    private void addAutosToChooser() {
        autoChooser.addRoutine("Nothing Auto", () -> autos.nothingAuto());
    }

    private Command driverRumble() {
        return new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1));
    }

    private Command operatorRumble() {
        return new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1));
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        CurrentAlliance.update();
        ShiftTracker.update();

        DogLog.forceNt.log("Alliance", CurrentAlliance.get().name());
        DogLog.forceNt.log("Match Phase", ShiftTracker.getCurrentMatchPhase());
        DogLog.forceNt.log("Hub Scorable", ShiftTracker.canScore());
        DogLog.forceNt.log("Driving Mode", driveFieldCentric ? "Field Centric" : "Robot Centric");
    }
}
