// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.util.AutoStartingPosition;

public class RobotContainer {
    private final Driver driver = Driver.MO;

    private enum Driver {
        MATT, MO, COACH_BOGDANANOV
    }

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private final RobotSystem robot;

    private final Autos autos;
    private final AutoChooser autoChooser = new AutoChooser();

    private boolean deployOverRetract = true;
    private String lastAutoName = "";

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

        //Preload already selected auto
        autos.preloadAuto(autoChooser.selectedCommand().getName());
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
                () -> driverController.y().getAsBoolean() && !deployOverRetract,
                () -> operatorController.b().getAsBoolean()
            );

            driverController.y().onTrue(new InstantCommand(() -> deployOverRetract = !deployOverRetract));
        }
        else {
            robot.setIntakeDefaultCommand(
                () -> driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(),
                () -> driverController.getRightTriggerAxis() > 0.1,
                () -> driverController.rightBumper().getAsBoolean(),
                () -> operatorController.b().getAsBoolean()
            );
        }

        driverController.rightStick().onTrue(robot.seedFieldCentric());
        driverController.leftBumper().whileTrue(robot.activatePrecisionDriving());

        driverController.start().onTrue(robot.forceResetOdometry());
        
        operatorController.rightBumper().whileTrue(robot.shoot());
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
        autoChooser.addRoutine("LeftBear", () -> autos.mareBetal());

        autoChooser.addRoutine("LeftTB", () -> autos.tb(AutoStartingPosition.LEFT));
        autoChooser.addRoutine("RightTB", () -> autos.tb(AutoStartingPosition.RIGHT));

        autoChooser.addRoutine("LeftTT", () -> autos.tt(AutoStartingPosition.LEFT));
        autoChooser.addRoutine("RightTT", () -> autos.tt(AutoStartingPosition.RIGHT));

        // autoChooser.addRoutine("TestAuto", () -> autos.testAuto());

        // autoChooser.addRoutine("LeftMiddle", () -> autos.middle(AutoStartingPosition.LEFT));
        // autoChooser.addRoutine("RightMiddle", () -> autos.middle(AutoStartingPosition.RIGHT));
    } 

    /*
     * Called repeatedly while the robot is disabled. 
     * Used to preload the selected auto routine's trajectory file to prevent lag at the start of the match.
     */
    public void disabledPeriodic() {
        String selectedName = autoChooser.selectedCommand().getName();
        if (selectedName != null && !selectedName.equals(lastAutoName)) {
            lastAutoName = selectedName;
            autos.preloadAuto(selectedName);
        }
    }

    //Used mostly for telemetry and logging general match info
    public void periodic() {
        AllianceUtility.update();
        ShiftTracker.update();
    }

    public double getRobotYawDegrees() {
        return robot.getRobotYawDegrees();
    }

    public void toggleDisabledLeds(boolean disable) {
        robot.toggleDisabledLeds(disable);
    }

    public void resetFuelCounter() {
        robot.resetFuelShotCount();
    }

    public void resetAfterAuto() {
        robot.resetAfterAuto();
    }
}