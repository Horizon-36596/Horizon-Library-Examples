// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.zenith.ExampleRobot;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.horizon36596.zenith.solverslib.AutoSpec;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/**
 * Zenith's high-fidelity sim for this robot: runs an auto's OpMode under SimLoop, against
 * {@link ExampleRobotSim}, for the autonomous period plus a margin, and writes
 * {@code TeamCode/build/sim/<auto>.trace.json} for {@code zenith sim} to read.
 *
 * <p>zenith.json's {@code sim.command} runs it with {@code -Dzenith.auto=<auto>}, which TeamCode/build.gradle
 * forwards into the test JVM. Without it, as in a plain {@code ./gradlew :TeamCode:testDebugUnitTest}, it
 * runs every auto in the deployed assets, so every auto is driven in the sim on every test run.
 *
 * <p>The real robot code runs: the generated OpMode's base class, {@code ExampleRobot}, the four
 * subsystems, the command registration, Pedro's follower with its mecanum drivetrain and Pinpoint
 * localizer, and the SolversLib scheduler. Only the hardware is fake.
 */
class ZenithAutoHeadlessTest {

    /** The FTC autonomous period, seconds. */
    static final double AUTO_PERIOD_S = 30;
    /** How long past the period the sim keeps running, seconds, so an overrunning auto shows how far it got. */
    static final double MARGIN_S = 2;
    /** SimLoop's default tick: 50 Hz, about a Control Hub loop. */
    static final double TICK_S = OpModeRunner.DEFAULT_TICK_SECONDS;
    /** Ticks between INIT and PLAY: half a second, long enough for the Pinpoint to calibrate its IMU. */
    static final int INIT_LOOP_TICKS = 25;
    /** The alliance the sim runs as. ChooseAlliance's default, RED. */
    static final String RUNNING_ALLIANCE = "RED";

    /** The names zenith.json's {@code {auto}} can stand for: a file stem, never a path. */
    private static final Pattern AUTO_NAME = Pattern.compile("[a-z0-9][a-z0-9-]*");

    private final File root = projectRoot();
    private final File assets = new File(root, "TeamCode/src/main/assets/autos");

    @AfterEach
    void clearBlackboard() {
        OpMode.blackboard.clear();
    }

    @Test
    void theAutoRunsToTheEndInTheSim() throws IOException {
        for (String auto : autosToRun()) {
            runAndTrace(auto);
        }
    }

    private void runAndTrace(String auto) throws IOException {
        assertTrue(AUTO_NAME.matcher(auto).matches(), "not an auto name: \"" + auto + "\"");
        AutoSpec spec = AutoSpec.load(AutoSource.directory(assets), auto);
        assertTrue(hasGeneratedOpMode(auto), "no OpMode in zenith/generated loads " + auto
                + "; run zenith deploy so the sim runs an auto the robot has");

        Json autoJson = readJson(new File(assets, auto + ".auto.json"));
        Json start = autoJson.get("start").opt("holds");
        int heldAtStart = start == null ? 0 : (int) start.optNum("pollen", 0);
        ExampleRobotSim sim = new ExampleRobotSim(readJson(new File(assets, "robot.json")),
                readJson(new File(root, zenithJson().str("field"))), heldAtStart);
        Pose2d startPose = spec.mirrorsInto(RUNNING_ALLIANCE) ? new ExampleRobot().mirror(spec.startPose) : spec.startPose;
        sim.placeAt(startPose.getX(), startPose.getY(), startPose.getHeading());

        File traceFile = new File(root, "TeamCode/build/sim/" + auto + ".trace.json");
        Files.deleteIfExists(traceFile.toPath());
        OpMode.blackboard.clear();
        OpMode.blackboard.put(TracedAuto.AUTO, auto);
        OpMode.blackboard.put(TracedAuto.SOURCE_DIR, assets.getPath());
        OpMode.blackboard.put(TracedAuto.TRACE_FILE, traceFile.getPath());
        OpMode.blackboard.put(TracedAuto.TICK_S, TICK_S);
        OpMode.blackboard.put(TracedAuto.DRIVE_PLANT, sim.drivePlant);

        long wallStart = System.nanoTime();
        OpModeRunResult result = OpModeRunner.forTeamClass(TracedAuto.class, sim.hardwareMap)
                // The robot code, SolversLib, Pedro and the Zenith runtime all read the sim clock. Zenith's
                // runtime has to be in the same loader as SolversLib, because its classes extend SolversLib's.
                .teamPackages("org.firstinspires.ftc.teamcode.", "com.seattlesolvers.solverslib.",
                        SimClockLoader.PEDRO_PATHING, "org.horizon36596.zenith.")
                .keepBlackboard(true)
                .tickSeconds(TICK_S)
                .initLoopTicks(INIT_LOOP_TICKS)
                .activeTicks((int) Math.round((AUTO_PERIOD_S + MARGIN_S) / TICK_S))
                .eachTick(sim::step)
                .run();
        double wallS = (System.nanoTime() - wallStart) / 1e9;

        assertTrue(traceFile.isFile(), "the run wrote no trace at " + traceFile);
        Json trace = readJson(traceFile);
        List<String> recorded = new ArrayList<String>();
        double lastEndS = 0;
        StringBuilder report = new StringBuilder();
        report.append(String.format(Locale.ROOT, "%s: %d ticks in %.2f s of real time%n", auto, result.ticks(), wallS));
        for (Json step : trace.get("steps").items()) {
            recorded.add(step.str("id"));
            assertFalse(step.get("interrupted").bool(), auto + ": step " + step.str("id") + " was interrupted");
            lastEndS = Math.max(lastEndS, step.num("endS"));
            report.append(String.format(Locale.ROOT, "  %-16s %7.3f -> %7.3f s%n", step.str("id"), step.num("startS"),
                    step.num("endS")));
        }
        report.append(String.format(Locale.ROOT, "  held at the end %d, launches %d, pollen left on the field %d%n",
                sim.held(), sim.launches(), sim.pollenOnField()));
        for (String warning : result.warnings()) {
            report.append("  SimLoop warning: ").append(warning).append('\n');
        }
        System.out.print(report);

        List<String> planned = new ArrayList<String>();
        for (AutoSpec.Step step : spec.steps) {
            planned.add(step.id);
        }
        assertEquals(planned, recorded, auto + ": the steps that ran");
        assertTrue(lastEndS < AUTO_PERIOD_S, auto + ": the last step ended at " + lastEndS + " s");
    }

    /** The auto zenith.json's sim command names, or every deployed auto when none is named. */
    private List<String> autosToRun() {
        String named = System.getProperty("zenith.auto");
        if (named != null && !named.isEmpty()) {
            return Collections.singletonList(named);
        }
        List<String> autos = new ArrayList<String>();
        String[] files = assets.list();
        if (files != null) {
            for (String file : files) {
                if (file.endsWith(".auto.json")) {
                    autos.add(file.substring(0, file.length() - ".auto.json".length()));
                }
            }
        }
        Collections.sort(autos);
        assertFalse(autos.isEmpty(), "no auto files in " + assets);
        return autos;
    }

    /** @return true when a class zenith deploy generated returns this auto's name from autoName() */
    private boolean hasGeneratedOpMode(String auto) throws IOException {
        File generated = new File(root, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/zenith/generated");
        File[] sources = generated.listFiles();
        if (sources == null) {
            return false;
        }
        for (File source : sources) {
            String text = new String(Files.readAllBytes(source.toPath()), StandardCharsets.UTF_8);
            if (text.contains("autoName() { return \"" + auto + "\"; }")) {
                return true;
            }
        }
        return false;
    }

    private Json zenithJson() throws IOException {
        return readJson(new File(root, "zenith.json"));
    }

    private static Json readJson(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            return Json.parse(in, file.getName());
        }
    }

    /** The folder with zenith.json in it; Gradle runs unit tests from the TeamCode folder. */
    private static File projectRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        while (dir != null && !new File(dir, "zenith.json").isFile()) {
            dir = dir.getParentFile();
        }
        if (dir == null) {
            throw new IllegalStateException("no zenith.json above " + System.getProperty("user.dir"));
        }
        return dir;
    }
}
