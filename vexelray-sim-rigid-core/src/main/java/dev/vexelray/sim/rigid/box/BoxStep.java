package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.Pass;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A whole {@link Boxes} step as data, as {@code SphereStep} is for spheres: which kernel, over which named buffers,
 * with how many invocations, in order. Nothing here runs anything.
 *
 * <p>A substep is {@link Boxes#predict}, then {@link Boxes#walls}, then {@link Boxes#velocity}: boxes meet the walls
 * and not yet each other. {@code oqx} to {@code oqw} hold how each box was turned when the substep began, {@code dx}
 * to {@code jz} what the walls moved it by and the angular impulse they gave it, and {@code lambda} and {@code rub}
 * each corner's normal and tangential impulse against each wall this substep. The parameters' {@code h} is the substep, so a step
 * advances {@code substeps · h} seconds.
 */
public final class BoxStep implements Buffered {

    public final int boxes;
    public final int substeps;

    private final Map<String, BufferSpec> buffers = new LinkedHashMap<>();
    private final Map<String, int[]> constants = new LinkedHashMap<>();
    private final List<Pass> step;

    public BoxStep(int boxes, int substeps) {
        if (boxes < 1 || substeps < 1) {
            throw new IllegalArgumentException("a step needs boxes and substeps, got " + boxes + " and " + substeps);
        }
        this.boxes = boxes;
        this.substeps = substeps;
        for (String field : Boxes.BOX) {
            buffers.put(field, new BufferSpec(field, Body.F32, boxes));
        }
        int[] identity = new int[boxes];
        java.util.Arrays.fill(identity, Float.floatToRawIntBits(1));
        constants.put("qw", identity);
        buffers.put("params", new BufferSpec("params", Body.F32, Boxes.PARAM_COUNT));
        buffers.put("lambda", new BufferSpec("lambda", Body.F32, Boxes.LAMBDA_WORDS * boxes));
        buffers.put("rub", new BufferSpec("rub", Body.F32, Boxes.RUB_WORDS * boxes));

        Pass predict = new Pass("predict", Boxes.predict(), Boxes.PREDICT_BUFFERS, List.of(Boxes.PREDICT_NAMES),
                boxes);
        Pass walls = new Pass("walls", Boxes.walls(), Boxes.WALLS_BUFFERS, List.of(Boxes.WALLS_NAMES), boxes);
        Pass velocity = new Pass("velocity", Boxes.velocity(), Boxes.VELOCITY_BUFFERS,
                List.of(Boxes.VELOCITY_NAMES), boxes);
        List<Pass> passes = new ArrayList<>();
        for (int sub = 0; sub < substeps; sub++) {
            passes.add(predict);
            passes.add(walls);
            passes.add(velocity);
        }
        step = List.copyOf(passes);
    }

    @Override
    public Map<String, BufferSpec> buffers() {
        return buffers;
    }

    /**
     * What to write into which buffers once, after they are cleared and before the first step: every orientation as
     * the identity, which a host that wants others writes after.
     */
    public Map<String, int[]> constants() {
        return constants;
    }

    /** One step: {@link #substeps} substeps. */
    public List<Pass> step() {
        return step;
    }
}
