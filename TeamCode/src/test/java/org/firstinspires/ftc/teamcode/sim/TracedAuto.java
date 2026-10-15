// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.sim;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.zenith.ExampleRobot;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.solverslib.AutoFromFile;
import org.horizon36596.zenith.solverslib.AutoTraceWriter;
import org.horizon36596.zenith.solverslib.ZenithRobot;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;

/**
 * The OpMode {@code zenith deploy} generates for an auto, with a trace recorder around it. Like the
 * generated class it is an {@link AutoFromFile} that builds {@link ExampleRobot}; the differences are only
 * what a sim needs:
 *
 * <ul>
 *   <li>the auto's name and file folder come from the test instead of being written into the class;</li>
 *   <li>the robot is an {@link ExampleRobot} whose {@code step} wraps every step in
 *       {@link AutoTraceWriter#traced}, the hook Zenith's runtime gives a sim for exactly this;</li>
 *   <li>every loop it records the follower's pose (the robot's belief) and the plant's pose (the truth),
 *       and when the OpMode ends it writes the trace.</li>
 * </ul>
 *
 * <p>SimLoop builds this class itself, inside its sim-clock class loader ({@code OpModeRunner.forTeamClass}),
 * so it cannot be handed objects by constructor. The test passes what it needs through
 * {@code OpMode.blackboard}, under the keys below, using only SDK, JDK and SimLoop types, which both sides
 * of the loader share. {@link System#nanoTime()} here, in the robot code, in Pedro and in the Zenith
 * runtime all read the sim clock.
 */
public final class TracedAuto extends AutoFromFile {

    /** The auto's file stem, a String. */
    static final String AUTO = "zenith.sim.auto";
    /** The folder the auto file and waypoints.json are read from, a String path. */
    static final String SOURCE_DIR = "zenith.sim.sourceDir";
    /** Where the trace is written, a String path. */
    static final String TRACE_FILE = "zenith.sim.traceFile";
    /** The sim's tick, seconds, a Double. */
    static final String TICK_S = "zenith.sim.tickS";
    /** The chassis plant, a MecanumDrivePlant, for the true pose. */
    static final String DRIVE_PLANT = "zenith.sim.drivePlant";

    private AutoTraceWriter trace;
    private MecanumDrivePlant plant;
    private long startNanos;
    private boolean started;

    @Override
    protected String autoName() {
        return (String) blackboard.get(AUTO);
    }

    @Override
    protected ZenithRobot createRobot() {
        return new ExampleRobot() {
            @Override
            public Command step(String id, Command body) {
                return trace.traced(id, body);
            }
        };
    }

    @Override
    public void initialize() {
        trace = new AutoTraceWriter(autoName(), (Double) blackboard.get(TICK_S));
        plant = (MecanumDrivePlant) blackboard.get(DRIVE_PLANT);
        setAutoSource(AutoSource.directory(new File((String) blackboard.get(SOURCE_DIR))));
        super.initialize();
    }

    /** Trace time 0 is the whistle: the moment the routine is scheduled. */
    @Override
    public void startRoutine() {
        startNanos = System.nanoTime();
        started = true;
        trace.tick(0);
        super.startRoutine();
        record();
    }

    @Override
    public void run() {
        trace.tick((System.nanoTime() - startNanos) / 1e9);
        super.run();
        record();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            try {
                trace.write(new File((String) blackboard.get(TRACE_FILE)));
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    private void record() {
        if (!started) {
            return;
        }
        Pose2d belief = currentPose();
        trace.pose(belief.getX(), belief.getY(), belief.getHeading());
        trace.truthPose(plant.getX(), plant.getY(), plant.getHeading());
    }
}
