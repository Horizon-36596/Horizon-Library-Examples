// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.zenith;

/**
 * The alliance the next auto runs as. The Choose alliance OpMode sets it before a match, and it
 * stays set until the robot controller app restarts. RED until someone chooses.
 */
public final class AllianceSelection {

    public static final String RED = "RED";
    public static final String BLUE = "BLUE";

    private static volatile String current = RED;

    private AllianceSelection() {}

    public static String current() {
        return current;
    }

    public static void choose(String alliance) {
        if (!RED.equals(alliance) && !BLUE.equals(alliance)) {
            throw new IllegalArgumentException("alliance must be RED or BLUE, got " + alliance);
        }
        current = alliance;
    }
}
