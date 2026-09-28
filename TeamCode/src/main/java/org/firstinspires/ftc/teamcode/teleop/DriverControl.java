// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.ExampleSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

/**
 * The driver-controlled period. The controls are in {@link DriverControls}. Every generated auto
 * selects this OpMode when it ends, through {@code deploy.preselectTeleOp} in zenith.json.
 */
@TeleOp(name = "Driver Control", group = "Match")
public class DriverControl extends CommandOpMode {

    private ExampleSubsystems robot;

    @Override
    public void initialize() {
        robot = ExampleSubsystems.create(hardwareMap);
        final GamepadEx driver = new GamepadEx(gamepad1);

        robot.drive.setDefaultCommand(new RunCommand(
                () -> robot.drive.driveManual(DriverControls.drive(driver)), robot.drive));
        DriverControls.bind(driver, robot.intake, robot.launcher, robot.hopper, System::nanoTime);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Hopper", robot.hopper.count() + " of " + HopperSubsystem.CAPACITY);
        telemetry.addData("Launcher", robot.launcher.ready() ? "ready" : "spinning up or idle");
        telemetry.update();
    }
}
