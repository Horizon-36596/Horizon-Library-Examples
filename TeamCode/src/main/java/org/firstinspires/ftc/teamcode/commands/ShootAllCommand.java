// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.horizon36596.zenith.RobotClock;

/**
 * Spins the launcher up and launches {@code count} pollen, one feeder stroke each. It takes
 * {@link #SPIN_UP_S} plus {@link #FEED_S} per pollen when the flywheel keeps up, which is the
 * {@code estimateS} of shootAll in robot.json: {@code 0.35 + count * 0.08}.
 *
 * <p>Time comes from the auto's {@link RobotClock}, so the same command takes the same time on the
 * robot and in a headless test. The launcher stays at launch speed afterwards; launcherIdle slows it.
 */
public class ShootAllCommand extends CommandBase {

    /** How the launches are spaced. The file names are lower case, as robot.json writes them. */
    public enum Cadence {
        /** One stroke after another, without waiting for the flywheel. */
        RAPID,
        /** Waits for the flywheel to be back at speed before each stroke. */
        PRECISE;

        /** @return the cadence a file names, {@code "rapid"} or {@code "precise"} */
        public static Cadence fromFile(String name) {
            if ("rapid".equals(name)) {
                return RAPID;
            }
            if ("precise".equals(name)) {
                return PRECISE;
            }
            throw new IllegalArgumentException("shootAll: cadence must be \"rapid\" or \"precise\", got \"" + name + "\"");
        }
    }

    /** PLACEHOLDER: flywheel spin-up before the first launch, seconds. */
    public static final double SPIN_UP_S = 0.35;
    /** PLACEHOLDER: one feeder stroke, push and return, seconds. */
    public static final double FEED_S = 0.08;

    private enum Phase { SPIN_UP, PUSH, RETURN, WAIT_READY, DONE }

    private final LauncherSubsystem launcher;
    private final int count;
    private final Cadence cadence;
    private final RobotClock clock;

    private Phase phase;
    private long phaseStartNanos;
    private int launched;

    /**
     * @param launcher the launcher that spins up and feeds
     * @param hopper   required too, so no other command moves pollen while a volley runs
     * @param count    how many pollen to launch, 1 to 4
     * @param cadence  how the launches are spaced
     * @param clock    the auto's clock
     */
    public ShootAllCommand(LauncherSubsystem launcher, HopperSubsystem hopper, int count, Cadence cadence,
            RobotClock clock) {
        if (count < 1 || count > HopperSubsystem.CAPACITY) {
            throw new IllegalArgumentException("shootAll: count must be 1 to " + HopperSubsystem.CAPACITY + ", got " + count);
        }
        this.launcher = launcher;
        this.count = count;
        this.cadence = cadence;
        this.clock = clock;
        addRequirements(launcher, hopper);
    }

    @Override
    public void initialize() {
        launched = 0;
        launcher.spinUp();
        enter(Phase.SPIN_UP);
    }

    @Override
    public void execute() {
        switch (phase) {
            case SPIN_UP:
                if (elapsedS() >= SPIN_UP_S && (cadence == Cadence.RAPID || launcher.ready())) {
                    push();
                }
                break;
            case PUSH:
                if (elapsedS() >= FEED_S / 2) {
                    launcher.feed(false);
                    enter(Phase.RETURN);
                }
                break;
            case RETURN:
                if (elapsedS() >= FEED_S / 2) {
                    launched++;
                    if (launched == count) {
                        enter(Phase.DONE);
                    } else if (cadence == Cadence.PRECISE && !launcher.ready()) {
                        enter(Phase.WAIT_READY);
                    } else {
                        push();
                    }
                }
                break;
            case WAIT_READY:
                if (launcher.ready()) {
                    push();
                }
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.feed(false);
    }

    /** @return how many pollen this run has launched so far */
    public int launched() {
        return launched;
    }

    private void push() {
        launcher.feed(true);
        enter(Phase.PUSH);
    }

    private void enter(Phase next) {
        phase = next;
        phaseStartNanos = clock.nanoTime();
    }

    private double elapsedS() {
        return (clock.nanoTime() - phaseStartNanos) / 1e9;
    }
}
