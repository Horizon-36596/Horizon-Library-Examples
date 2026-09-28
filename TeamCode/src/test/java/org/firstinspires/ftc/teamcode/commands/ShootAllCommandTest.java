// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.testing.FakeDevice;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class ShootAllCommandTest {

    private static final long LOOP_NANOS = 5_000_000L;

    private final LauncherSubsystem launcher = new LauncherSubsystem(FakeDevice.motor(), FakeDevice.servo());
    private final HopperSubsystem hopper = new HopperSubsystem(FakeDevice.beamBreak(true),
            FakeDevice.beamBreak(true), FakeDevice.beamBreak(true), FakeDevice.beamBreak(true));
    private long nowNanos;

    @AfterEach
    void resetScheduler() {
        CommandScheduler.getInstance().reset();
    }

    @Test
    void takesAsLongAsRobotJsonEstimates() {
        for (ShootAllCommand.Cadence cadence : ShootAllCommand.Cadence.values()) {
            for (int count = 1; count <= HopperSubsystem.CAPACITY; count++) {
                double estimateS = 0.35 + count * 0.08;
                double tookS = run(new ShootAllCommand(launcher, hopper, count, cadence, () -> nowNanos));
                assertEquals(estimateS, tookS, 2 * LOOP_NANOS / 1e9, cadence + " x" + count);
            }
        }
    }

    @Test
    void launchesTheCountItWasGiven() {
        ShootAllCommand shoot = new ShootAllCommand(launcher, hopper, 3, ShootAllCommand.Cadence.RAPID, () -> nowNanos);
        run(shoot);
        assertEquals(3, shoot.launched());
        assertTrue(launcher.ready(), "the launcher stays at speed after a volley");
    }

    @Test
    void refusesACountTheHopperCannotHold() {
        assertThrows(IllegalArgumentException.class,
                () -> new ShootAllCommand(launcher, hopper, 5, ShootAllCommand.Cadence.PRECISE, () -> nowNanos));
        assertThrows(IllegalArgumentException.class, () -> ShootAllCommand.Cadence.fromFile("fast"));
    }

    /** Runs the command the way the scheduler would, and returns how long it took in seconds. */
    private double run(ShootAllCommand command) {
        long startNanos = nowNanos;
        command.initialize();
        for (int loop = 0; loop < 1000; loop++) {
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                return (nowNanos - startNanos) / 1e9;
            }
            nowNanos += LOOP_NANOS;
        }
        throw new AssertionError("shootAll did not finish");
    }
}
