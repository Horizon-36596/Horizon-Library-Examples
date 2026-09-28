// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.testing.FakeDevice;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriverControlsTest {

    private final Gamepad gamepad = new Gamepad();
    private final DigitalChannel[] slots = {
            FakeDevice.beamBreak(false), FakeDevice.beamBreak(false),
            FakeDevice.beamBreak(false), FakeDevice.beamBreak(false)};
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;
    private long nowNanos;

    @BeforeEach
    void bindTheControls() {
        CommandScheduler.getInstance().reset();
        intake = new IntakeSubsystem(FakeDevice.motor());
        launcher = new LauncherSubsystem(FakeDevice.motor(), FakeDevice.servo());
        HopperSubsystem hopper = new HopperSubsystem(slots);
        DriverControls.bind(new GamepadEx(gamepad), intake, launcher, hopper, () -> nowNanos);
    }

    @AfterEach
    void resetScheduler() {
        CommandScheduler.getInstance().reset();
    }

    @Test
    void sticksBecomeRobotCentricPowers() {
        DrivePowers powers = DriverControls.mix(0.8, 0.5, 0.3);
        assertEquals(0.8, powers.forward(), 1e-9);
        assertEquals(-0.5, powers.strafe(), 1e-9, "stick right strafes right, which is negative to Pedro");
        assertEquals(-0.3, powers.turn(), 1e-9, "stick right turns clockwise, which is negative to Pedro");

        DrivePowers resting = DriverControls.mix(0.03, -0.04, 0.02);
        assertEquals(0, resting.forward(), 0);
        assertEquals(0, resting.strafe(), 0);
        assertEquals(0, resting.turn(), 0);
    }

    @Test
    void theRightBumperRunsTheIntakeUntilReleasedOrFull() {
        gamepad.right_bumper = true;
        loop();
        assertEquals(IntakeSubsystem.State.FORWARD, intake.frontState());

        gamepad.right_bumper = false;
        loop();
        assertEquals(IntakeSubsystem.State.STOP, intake.frontState());

        gamepad.right_bumper = true;
        loop();
        for (DigitalChannel slot : slots) {
            FakeDevice.setBeamBroken(slot, true);
        }
        loop();
        assertEquals(IntakeSubsystem.State.STOP, intake.frontState(), "a full hopper stops the intake");
    }

    @Test
    void theLeftBumperClearsAJam() {
        gamepad.left_bumper = true;
        loop();
        assertEquals(IntakeSubsystem.State.REVERSE, intake.frontState());
    }

    @Test
    void aLaunchesAndBIdlesTheLauncher() {
        gamepad.a = true;
        loop();
        assertTrue(launcher.ready(), "A spins the launcher up to launch");
        assertTrue(CommandScheduler.getInstance().requiring(launcher) != null, "a volley is running");

        gamepad.a = false;
        gamepad.b = true;
        loop();
        assertFalse(launcher.ready(), "B drops the launcher to idle");
        assertEquals(null, CommandScheduler.getInstance().requiring(launcher), "B also ended the volley");
    }

    /** One scheduler loop, 20 ms apart, as the TeleOp runs it. */
    private void loop() {
        CommandScheduler.getInstance().run();
        nowNanos += 20_000_000L;
    }
}
