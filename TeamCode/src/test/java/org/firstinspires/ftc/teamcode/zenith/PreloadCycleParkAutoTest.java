// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.zenith;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.testing.FakeDevice;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.horizon36596.zenith.solverslib.AutoBuilder;
import org.horizon36596.zenith.solverslib.AutoSpec;
import org.horizon36596.zenith.solverslib.NamedCommands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Loads preload-cycle-park the way the generated OpMode does, from the deployed assets, and builds
 * and runs it through the Zenith runtime against this robot's own command registration.
 */
class PreloadCycleParkAutoTest {

    private static final String AUTO = "preload-cycle-park";
    private static final double AUTO_PERIOD_S = 30;

    private final File root = projectRoot();
    private final File assets = new File(root, "TeamCode/src/main/assets/autos");

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;

    @BeforeEach
    void registerTheRobotsCommands() {
        CommandGroupBase.clearGroupedCommands();
        drive = new DriveSubsystem(new Follower(null, null, null), null);
        intake = new IntakeSubsystem(FakeDevice.motor());
        launcher = new LauncherSubsystem(FakeDevice.motor(), FakeDevice.servo());
        HopperSubsystem hopper = new HopperSubsystem(FakeDevice.beamBreak(false), FakeDevice.beamBreak(false),
                FakeDevice.beamBreak(false), FakeDevice.beamBreak(false));
        NamedCommands.reset();
        ExampleCommands.register(drive, intake, launcher, hopper);
    }

    @AfterEach
    void resetScheduler() {
        CommandScheduler.getInstance().reset();
    }

    @Test
    void theRobotRegistersExactlyTheNamesInRobotJson() throws IOException {
        Json robot;
        try (InputStream in = new FileInputStream(new File(root, zenithJson().str("robot")))) {
            robot = Json.parse(in, "robot.json");
        }
        assertEquals(names(robot.get("commands")), NamedCommands.commandNames());
        assertEquals(names(robot.get("conditions")), NamedCommands.conditionNames());
    }

    @Test
    void theAutoBuildsAndRunsInsideTheAutoPeriod() throws IOException {
        AutoSpec spec = AutoSpec.load(AutoSource.directory(assets), AUTO);
        TestAuto auto = new TestAuto(drive, spec.startPose);
        Command routine = AutoBuilder.build(spec, auto);

        double tookS = auto.run(routine, AUTO_PERIOD_S);

        assertTrue(tookS < AUTO_PERIOD_S, "took " + tookS + " s");
        assertEquals(Arrays.asList("toShoot", "shootPreload", "toGarden", "sweepGarden", "stopIntake",
                "toShootNorth", "shootCollected", "park"), auto.startedSteps);
        assertEquals(IntakeSubsystem.State.STOP, intake.frontState());
        assertFalse(launcher.ready(), "the park marker drops the launcher to idle");
    }

    @Test
    void theDeployedFilesAreTheSourceFiles() throws IOException {
        for (String name : new String[] {AUTO + ".auto.json", "waypoints.json", "robot.json"}) {
            assertArrayEquals(read(new File(root, "autos/" + name)), read(new File(assets, name)), name);
        }
        File field = new File(root, zenithJson().str("field"));
        assertArrayEquals(read(field), read(new File(assets, field.getName())), "the field file");
    }

    @Test
    void theGeneratedJavaIsFromTheCurrentFile() throws IOException, NoSuchAlgorithmException {
        byte[] digest = MessageDigest.getInstance("SHA-256").digest(read(new File(root, "autos/" + AUTO + ".auto.json")));
        String sha = String.format("%02x%02x%02x%02x", digest[0], digest[1], digest[2], digest[3]);
        String generated = new String(read(new File(root,
                "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/zenith/PreloadCycleParkGenerated.java")),
                StandardCharsets.UTF_8);
        assertTrue(generated.contains("(sha " + sha + ")"), "run zenith codegen and zenith deploy again");
    }

    private static List<String> names(Json entries) {
        List<String> names = new ArrayList<String>();
        for (Json entry : entries.items()) {
            names.add(entry.str("name"));
        }
        Collections.sort(names);
        return names;
    }

    private Json zenithJson() throws IOException {
        try (InputStream in = new FileInputStream(new File(root, "zenith.json"))) {
            return Json.parse(in, "zenith.json");
        }
    }

    private static byte[] read(File file) throws IOException {
        try (InputStream in = new FileInputStream(file)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            for (int n; (n = in.read(buffer)) > 0; ) {
                out.write(buffer, 0, n);
            }
            return out.toByteArray();
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
