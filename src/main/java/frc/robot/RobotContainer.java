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
import frc.robot.subsystems.DriveSubsystem.FieldPose;
import frc.robot.subsystems.ShootingSuperstructure.PassingTarget;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final CommandXboxController testController = new CommandXboxController(3);

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
            () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
            () -> driverController.y().getAsBoolean()
        );

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.povDown().onTrue(new InstantCommand(() -> driveFieldCentric = !driveFieldCentric));

        operatorController.rightBumper().whileTrue(robot.shoot());
        operatorController.leftBumper().whileTrue(robot.clearTransfer());

        operatorController.a().whileTrue(robot.rotateRobotToShoot());
        operatorController.x().whileTrue(robot.activateSlowMo());
        driverController.a().whileTrue(robot.driveToPose(FieldPose.CLIMB_LEFT));

        //Passing target changing
        operatorController.povLeft().onTrue(robot.setPassingTarget(PassingTarget.LEFT));
        operatorController.povUp().onTrue(robot.setPassingTarget(PassingTarget.MIDDLE));
        operatorController.povRight().onTrue(robot.setPassingTarget(PassingTarget.RIGHT));

        testController.back().and(driverController.y()).whileTrue(robot.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        testController.back().and(driverController.x()).whileTrue(robot.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        testController.back().and(driverController.a()).whileTrue(robot.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
        testController.back().and(driverController.b()).whileTrue(robot.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    /*
     * Add all our working autonomous routines to the chooser for selecting on Elastic
     */
    private void addAutosToChooser() {
        // autoChooser.addRoutine("OneCycleWin", () -> autos.oneCycle());
        autoChooser.addRoutine("RightOneCyclePlusOutpost", () -> autos.rightOneCyclePlusOutpost());
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