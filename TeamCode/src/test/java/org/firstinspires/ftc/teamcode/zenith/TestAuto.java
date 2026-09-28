// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.zenith;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.horizon36596.zenith.solverslib.AutoContext;

import java.util.ArrayList;
import java.util.List;

/**
 * A running auto with no drivetrain behind it, for building and running the routine on the JVM.
 * A path "drives" for {@link #PATH_LOOPS} loops and reports steady progress through a Pedro follower
 * with no hardware, so markers fire. Time is {@link #nowNanos}, advanced one loop at a time.
 */
final class TestAuto implements AutoContext {

    static final long LOOP_NANOS = 20_000_000L;
    static final int PATH_LOOPS = 10;

    final List<String> startedSteps = new ArrayList<String>();
    long nowNanos;

    private final Subsystem drive;
    private final Pose2d livePose;
    private boolean following;
    private int pathIndex;
    private double curveCompletion;

    private final Follower follower = new Follower(null, null, null) {
        @Override
        public boolean following() {
            return following;
        }

        @Override
        public int pathIndex() {
            return pathIndex;
        }

        @Override
        public double curveCompletion() {
            return curveCompletion;
        }
    };

    TestAuto(Subsystem drive, Pose2d livePose) {
        this.drive = drive;
        this.livePose = livePose;
    }

    @Override
    public Follower follower() {
        return follower;
    }

    @Override
    public Subsystem drive() {
        return drive;
    }

    @Override
    public long nanoTime() {
        return nowNanos;
    }

    /** The file is written for RED and this runs as RED, so nothing is mirrored. */
    @Override
    public Pose2d alliance(Pose2d filePose) {
        return filePose;
    }

    @Override
    public Pose2d currentPose() {
        return livePose;
    }

    @Override
    public Command followPath(Path path, double speedFraction) {
        CommandBase drivePath = new CommandBase() {
            private int loops;

            @Override
            public void initialize() {
                loops = 0;
                following = true;
                pathIndex = 0;
                curveCompletion = 0;
            }

            @Override
            public void execute() {
                loops++;
                curveCompletion = Math.min(1.0, (double) loops / PATH_LOOPS);
                following = loops < PATH_LOOPS;
            }

            @Override
            public boolean isFinished() {
                return loops >= PATH_LOOPS;
            }

            @Override
            public void end(boolean interrupted) {
                following = false;
            }
        };
        drivePath.addRequirements(drive);
        return drivePath;
    }

    /** Records each step's id as the step starts, so a test can check the order they ran in. */
    @Override
    public Command step(final String id, final Command body) {
        CommandBase traced = new CommandBase() {
            @Override
            public void initialize() {
                startedSteps.add(id);
                body.initialize();
            }

            @Override
            public void execute() {
                body.execute();
            }

            @Override
            public boolean isFinished() {
                return body.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                body.end(interrupted);
            }
        };
        traced.addRequirements(body.getRequirements().toArray(new Subsystem[0]));
        return traced;
    }

    /** Runs a command to the end the way the scheduler would, one loop at a time. */
    double run(Command command, double limitS) {
        long startNanos = nowNanos;
        command.initialize();
        while ((nowNanos - startNanos) / 1e9 < limitS) {
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                return (nowNanos - startNanos) / 1e9;
            }
            nowNanos += LOOP_NANOS;
        }
        throw new AssertionError("the routine was still running after " + limitS + " s");
    }
}
