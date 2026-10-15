// Copyright (c) 2026 Horizon (FTC 36596). MIT licence: see LICENSE at the repository root.

package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.hardware.PedroSetup;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.zenith.Json;

import java.util.ArrayList;
import java.util.List;

/**
 * The example robot in SimLoop: a fake device under every name in {@link HardwareNames}, a mecanum plant
 * behind the four drive motors, a fake goBILDA Pinpoint fed from that plant, and just enough of the
 * pollen path (intake, hopper, feeder) for the auto's commands and conditions to mean something.
 *
 * <p>Nothing in the robot code knows about this class. The OpMode builds its subsystems from
 * {@link #hardwareMap} exactly as it would from the Control Hub's.
 *
 * <p><b>Frames.</b> The plant's pose is kept in Zenith's field frame (inches, x and y as the field file
 * draws them, heading counter-clockwise from +x), which is also Pedro's frame here, so the plant's pose
 * is the robot's true field pose. The Pinpoint only ever sees the plant's motion, never its origin, so
 * its pose is whatever Pedro wrote at init plus the motion since.
 *
 * <p>Every number below that is not read from robot code or from {@code autos/} is invented: a plausible
 * round figure, marked PLACEHOLDER, like the rest of this repository's numbers.
 */
public final class ExampleRobotSim {

    /** PLACEHOLDER: drive encoder counts per inch of travel, a 435 rpm motor (384.5 cpr) on a 104 mm wheel. */
    static final double DRIVE_TICKS_PER_INCH = 30;
    /** PLACEHOLDER: counts per revolution of a 435 rpm drive motor's output shaft. */
    static final double DRIVE_TICKS_PER_REV = 384.5;
    /**
     * PLACEHOLDER: how fast a drive motor's speed follows its power, 1/s. 5 is a 0.2 s time constant;
     * SimLoop's DrivetrainSimConfig says values above about 6 start to oscillate.
     */
    static final double DRIVE_RESPONSE_PER_S = 5;

    /** PLACEHOLDER: flywheel free speed, ticks/s: a 6000 rpm motor with a 28 count encoder. */
    static final double FLYWHEEL_MAX_TICKS_PER_S = 2800;
    /** PLACEHOLDER: counts per revolution of the flywheel motor. */
    static final double FLYWHEEL_TICKS_PER_REV = 28;
    /** PLACEHOLDER: how fast the flywheel follows its command, 1/s. */
    static final double FLYWHEEL_RESPONSE_PER_S = 8;

    /** PLACEHOLDER: intake roller free speed, ticks/s: a 1150 rpm motor with a 145.1 count encoder. */
    static final double INTAKE_MAX_TICKS_PER_S = 2780;
    /** PLACEHOLDER: counts per revolution of the intake motor. */
    static final double INTAKE_TICKS_PER_REV = 145.1;

    /**
     * PLACEHOLDER: the feeder servo position past which a stroke has pushed one pollen into the flywheel.
     * Halfway between LauncherSubsystem's rest (0.30) and push (0.65) positions.
     */
    static final double FEEDER_LAUNCH_POSITION = 0.475;

    /** PLACEHOLDER: odometry pod offsets, the same as PedroSetup's, inches. */
    static final double X_POD_OFFSET_IN = -3.5;
    static final double Y_POD_OFFSET_IN = -6.0;

    private static final double MM_PER_IN = 25.4;

    /** The chassis. Top speeds and coast rates are the same numbers Pedro is told, so the two agree. */
    static final DrivetrainSimConfig DRIVETRAIN = new DrivetrainSimConfig() {
        /** PLACEHOLDER: left-right wheel centre distance, inches. */
        @Override public double trackWidth() { return 13; }
        /** PLACEHOLDER: front-back wheel centre distance, inches. */
        @Override public double wheelBase() { return 12; }
        /** PLACEHOLDER: a 104 mm mecanum wheel, inches. */
        @Override public double wheelRadius() { return 2.05; }
        @Override public double ticksPerInch() { return DRIVE_TICKS_PER_INCH; }
        /** PedroSetup's forward top speed, as wheel ticks per second. */
        @Override public double maxVelocityTicksPerSecond() {
            return PedroSetup.MAX_FORWARD_IN_PER_S * DRIVE_TICKS_PER_INCH;
        }
        /** PedroSetup's strafe top speed, as wheel ticks per second. */
        @Override public double maxLateralVelocityTicksPerSecond() {
            return PedroSetup.MAX_STRAFE_IN_PER_S * DRIVE_TICKS_PER_INCH;
        }
        @Override public double maxAccel() { return DRIVE_RESPONSE_PER_S; }
        @Override public double naturalForwardDecelerationInchesPerSecondSquared() {
            return PedroSetup.FORWARD_DECEL_IN_PER_S2;
        }
        @Override public double naturalLateralDecelerationInchesPerSecondSquared() {
            return PedroSetup.STRAFE_DECEL_IN_PER_S2;
        }
        /**
         * Which way each motor is bolted in, [FL, FR, BL, BR]. PedroSetup reverses the left pair in
         * software, so on the metal the left pair is the mirrored one.
         */
        @Override public double[] wheelMountingSigns() { return new double[] {-1, 1, -1, 1}; }
        /** Half of the 144 in field. */
        @Override public double fieldHalfWidth() { return 72; }
        @Override public double fieldHalfHeight() { return 72; }
    };

    /** The hardware map the OpMode is handed. */
    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();

    /** [frontLeft, frontRight, backLeft, backRight], the order the plant reads them in. */
    public final FakeMotor[] driveMotors = new FakeMotor[4];
    public final FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
    public final FakeMotor intake = new FakeMotor(INTAKE_MAX_TICKS_PER_S, DRIVE_RESPONSE_PER_S, INTAKE_TICKS_PER_REV);
    public final FakeMotor flywheel =
            new FakeMotor(FLYWHEEL_MAX_TICKS_PER_S, FLYWHEEL_RESPONSE_PER_S, FLYWHEEL_TICKS_PER_REV);
    public final FakeServo feeder = new FakeServo();
    public final FakeDigitalChannel[] hopperSlots = new FakeDigitalChannel[4];

    /** The chassis physics. Its pose is the robot's true field pose. */
    public final MecanumDrivePlant drivePlant;

    private final Mouth mouth;
    private final List<double[]> pollenOnField = new ArrayList<double[]>();
    private int held;
    private int launches;
    private boolean feederWasOut;

    /**
     * @param robotJson the robot file, for the intake mouth and the hopper's capacity
     * @param fieldJson the field file, for where the pollen lie
     * @param held      how many pollen the robot holds at the start
     */
    public ExampleRobotSim(Json robotJson, Json fieldJson, int held) {
        String[] driveNames = {HardwareNames.FRONT_LEFT, HardwareNames.FRONT_RIGHT, HardwareNames.BACK_LEFT,
                HardwareNames.BACK_RIGHT};
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new FakeMotor(DRIVETRAIN.maxVelocityTicksPerSecond(), DRIVE_RESPONSE_PER_S,
                    DRIVE_TICKS_PER_REV);
            hardwareMap.register(driveNames[i], driveMotors[i]);
        }
        // The pods sit where PedroSetup tells the Pinpoint they sit, and count the way it says they count.
        pinpoint.setPhysicalPodMount(X_POD_OFFSET_IN * MM_PER_IN, Y_POD_OFFSET_IN * MM_PER_IN,
                GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD,
                FakeGoBildaPinpoint.DEFAULT_TICKS_PER_MM);
        hardwareMap.register(HardwareNames.PINPOINT, pinpoint);
        hardwareMap.register(HardwareNames.INTAKE, intake);
        hardwareMap.register(HardwareNames.LAUNCHER, flywheel);
        hardwareMap.register(HardwareNames.FEEDER, feeder);
        String[] slotNames = {HardwareNames.HOPPER_SLOT_1, HardwareNames.HOPPER_SLOT_2, HardwareNames.HOPPER_SLOT_3,
                HardwareNames.HOPPER_SLOT_4};
        for (int i = 0; i < 4; i++) {
            hopperSlots[i] = new FakeDigitalChannel();
            hardwareMap.register(slotNames[i], hopperSlots[i]);
        }
        drivePlant = new MecanumDrivePlant(driveMotors, DRIVETRAIN);

        mouth = new Mouth(robotJson.get("mouths").at(0));
        for (Json element : fieldJson.get("elements").items()) {
            // Loose pollen only: one with a container is inside something the intake cannot reach.
            Json container = element.opt("container");
            if ("pollen".equals(element.optText("kind", "")) && (container == null || !container.isString())) {
                pollenOnField.add(new double[] {element.num("xIn"), element.num("yIn"), element.num("radiusIn")});
            }
        }
        this.held = held;
        updateHopperSensors();
    }

    /** Puts the chassis at a field pose without driving there, and tells the Pinpoint it was not motion. */
    public void placeAt(double xIn, double yIn, double headingRad) {
        drivePlant.setPose(xIn, yIn, headingRad);
        pinpoint.attachChassisAt(xIn * MM_PER_IN, yIn * MM_PER_IN, headingRad);
    }

    /**
     * The world's response to one tick of robot code. OpModeRunner calls it after the OpMode's loop and
     * after every fake device has ticked, so the motors' speeds are this tick's.
     *
     * @param dt the tick, seconds
     */
    public void step(double dt) {
        drivePlant.update(dt);
        pinpoint.moveChassisTo(drivePlant.getX() * MM_PER_IN, drivePlant.getY() * MM_PER_IN, drivePlant.getHeading());

        // One pollen leaves the hopper on each feeder stroke, if there is one to leave.
        boolean feederOut = feeder.getPosition() >= FEEDER_LAUNCH_POSITION;
        if (feederOut && !feederWasOut && held > 0) {
            held--;
            launches++;
        }
        feederWasOut = feederOut;

        // A pollen the running intake's mouth touches is taken in, while the hopper has room.
        if (intake.getPower() > 0) {
            for (int i = pollenOnField.size() - 1; i >= 0 && held < hopperSlots.length; i--) {
                if (mouth.touches(pollenOnField.get(i), drivePlant.getX(), drivePlant.getY(), drivePlant.getHeading())) {
                    pollenOnField.remove(i);
                    held++;
                }
            }
        }
        updateHopperSensors();
    }

    /** @return how many pollen the hopper holds */
    public int held() {
        return held;
    }

    /** @return how many pollen the feeder has pushed into the flywheel */
    public int launches() {
        return launches;
    }

    /** @return how many pollen are still lying on the field */
    public int pollenOnField() {
        return pollenOnField.size();
    }

    /** A slot's beam is broken while it holds a pollen; slots fill from the first. */
    private void updateHopperSensors() {
        for (int i = 0; i < hopperSlots.length; i++) {
            hopperSlots[i].setSimulatedInputState(i >= held);
        }
    }

    /** The intake's opening, from robot.json's mouths entry: a rectangle in the robot's frame. */
    private static final class Mouth {
        final double offsetXIn;
        final double offsetYIn;
        final double halfWidthIn;
        final double halfDepthIn;

        Mouth(Json mouth) {
            offsetXIn = mouth.get("offsetIn").num("xIn");
            offsetYIn = mouth.get("offsetIn").num("yIn");
            halfWidthIn = mouth.num("widthIn") / 2;
            halfDepthIn = mouth.num("depthIn") / 2;
        }

        /** @return true when a pollen of the given radius overlaps the mouth of a robot at this pose */
        boolean touches(double[] pollen, double xIn, double yIn, double headingRad) {
            double dx = pollen[0] - xIn;
            double dy = pollen[1] - yIn;
            double forward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
            double left = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
            return Math.abs(forward - offsetXIn) <= halfDepthIn + pollen[2]
                    && Math.abs(left - offsetYIn) <= halfWidthIn + pollen[2];
        }
    }
}
