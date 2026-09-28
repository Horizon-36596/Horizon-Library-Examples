// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.zenith.AllianceSelection;

/** Run before a match: press B for RED or X for BLUE, then stop. Every auto then runs as that alliance. */
@TeleOp(name = "Choose alliance", group = "Setup")
public class ChooseAlliance extends LinearOpMode {

    @Override
    public void runOpMode() {
        while (!isStopRequested()) {
            if (gamepad1.b) {
                AllianceSelection.choose(AllianceSelection.RED);
            } else if (gamepad1.x) {
                AllianceSelection.choose(AllianceSelection.BLUE);
            }
            telemetry.addData("Alliance", AllianceSelection.current());
            telemetry.addLine("B for RED, X for BLUE, then stop this OpMode.");
            telemetry.update();
            idle();
        }
    }
}
