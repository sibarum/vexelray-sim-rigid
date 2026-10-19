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
 * <p>So far a substep is only {@link Boxes#predict}: nothing touches a box yet. The parameters' {@code h} is the
 * substep, so a step advances {@code substeps · h} seconds.
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

        Pass predict = new Pass("predict", Boxes.predict(), Boxes.PREDICT_BUFFERS, List.of(Boxes.PREDICT_NAMES),
                boxes);
        List<Pass> passes = new ArrayList<>();
        for (int sub = 0; sub < substeps; sub++) {
            passes.add(predict);
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
