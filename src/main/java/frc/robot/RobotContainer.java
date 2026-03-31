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
import frc.robot.Autos.AutoPosition;
//import frc.robot.subsystems.LEDSubsystem.DisplayMode;


public class RobotContainer {
    private final Driver driver = Driver.MO;

    private enum Driver {
        MATT, MO, BOGDANANOV
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
            .withCaptureDs(false)
            .withLogExtras(false)
            .withCaptureConsole(true)
        );

        robot = new RobotSystem(driverController, operatorController);
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

        operatorController.povUp()
            .whileTrue(new InstantCommand(() -> robot.changeRPMOffset(25)));
        operatorController.povDown()
            .whileTrue(new InstantCommand(() -> robot.changeRPMOffset(-25)));
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