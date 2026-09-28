// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;

/** The roller intake across the front of the robot. The robot has no back intake. */
public class IntakeSubsystem extends SubsystemBase {

    /** Which intake a command addresses. The names match the setIntake command in robot.json. */
    public enum Side { FRONT, BACK, BOTH }

    /** What the intake does. The names match the setIntake command in robot.json. */
    public enum State { STOP, FORWARD, REVERSE }

    /** PLACEHOLDER: full power inward. */
    static final double FORWARD_POWER = 1.0;
    /** PLACEHOLDER: a gentler reverse, to clear a jam without throwing pollen. */
    static final double REVERSE_POWER = -0.6;

    private final DcMotorEx front;
    private State frontState = State.STOP;

    public IntakeSubsystem(DcMotorEx front) {
        this.front = front;
        front.setPower(0);
    }

    public static IntakeSubsystem create(HardwareMap hardwareMap) {
        return new IntakeSubsystem(hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE));
    }

    /**
     * Sets the intake on one side. FRONT and BOTH run the front intake; BACK does nothing, because this
     * robot has no back intake.
     */
    public void set(Side side, State state) {
        if (side == Side.BACK) {
            return;
        }
        frontState = state;
        front.setPower(powerFor(state));
    }

    public State frontState() {
        return frontState;
    }

    /** @return a command that sets the intake and finishes at once */
    public Command setCommand(final Side side, final State state) {
        return new InstantCommand(() -> set(side, state), this);
    }

    static double powerFor(State state) {
        switch (state) {
            case FORWARD:
                return FORWARD_POWER;
            case REVERSE:
                return REVERSE_POWER;
            default:
                return 0;
        }
    }
}
