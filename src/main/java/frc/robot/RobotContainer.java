// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.RobotSystem.DrivingMode;
import frc.robot.subsystems.DriveSubsystem.FieldPose;
import frc.robot.subsystems.ShootingSuperstructure.PassingTarget;


public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser;
    
    public RobotContainer() {
        //TODO: Enable for competition
        // DogLog.setOptions(new DogLogOptions()
        //     .withCaptureDs(true)
        //     .withLogExtras(true)
        //     .withCaptureConsole(true)
        //     .withCaptureNt(true)
        // );

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

        robot.setIntakeDefaultCommand(
            () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
            () -> driverController.rightTrigger().getAsBoolean(),
            () -> driverController.rightBumper().getAsBoolean()
        );

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.povDown().onTrue(new InstantCommand(() -> robot.toggleDrivingMode()));

        operatorController.rightBumper().whileTrue(robot.shoot());
        operatorController.leftBumper().whileTrue(robot.clearTransfer());
        
        operatorController.b().onTrue(robot.agitate());

        operatorController.a().whileTrue(robot.keepRobotLockedWithTurret());
        operatorController.x().whileTrue(robot.activatePrecisionDriving());
        driverController.a().whileTrue(robot.driveToPose(FieldPose.CLIMB_LEFT));

        //Passing target changing
        operatorController.povLeft().onTrue(robot.setPassingTarget(PassingTarget.LEFT));
        operatorController.povUp().onTrue(robot.setPassingTarget(PassingTarget.MIDDLE));
        operatorController.povRight().onTrue(robot.setPassingTarget(PassingTarget.RIGHT));

        operatorController.povDown().onTrue(robot.homeClimber());
        operatorController.y().onTrue(robot.toggleClimb());
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
        robot.disabledLeds();
    }

    public void resetAfterAuto() {
        CommandScheduler.getInstance().schedule(robot.deactivateShooter());
    }
}