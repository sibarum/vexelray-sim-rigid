package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.Fullscreen;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.vulkan.present.BoundStorageBuffer;
import dev.vexelray.vulkan.present.GraphicsPipeline;
import dev.vexelray.vulkan.present.SampledColorTarget;

/**
 * Spheres on screen, ray-traced straight from a finished step into a target a node shows.
 *
 * <p>The state is never copied to the host for this: the view binds the ring slot ({@link ShownRing}) the newest
 * finished step was kept in, and reads it where it is, so the cost of the picture is one fullscreen pass. That works
 * only because the simulation computes on the device the application draws on. The draw waits for the step's
 * timeline value inside the GPU, which the step reached before it was published, so the frame never waits for
 * physics: it draws the newest step there is.
 *
 * <p>The camera is an {@link Orbit}, of a box scaled so its longest side is two units across and centred on the
 * origin; this view turns that into the simulation's own coordinates, in metres, and the shader works there.
 *
 * <p>Main thread only, as Vulkan is. The target is minted by the application, which closes it; the pipeline and the
 * binding are this view's.
 */
public final class SphereView implements AutoCloseable {

    private final Node node;
    private final int pixels;
    private final Orbit orbit;

    private SampledColorTarget target;
    private GraphicsPipeline pipeline;
    /** The bindings of the generation drawn last, one a slot, made as each slot is first drawn. */
    private final BoundStorageBuffer[] slots = new BoundStorageBuffer[ShownRing.SLOTS];
    private long generation;

    /**
     * @param node   where the picture is shown; sized by layout, and the picture scales into it
     * @param pixels the side of the square target
     * @param orbit  the camera, which something else turns
     */
    public SphereView(Node node, int pixels, Orbit orbit) {
        this.node = node;
        this.pixels = pixels;
        this.orbit = orbit;
    }

    /**
     * Draws {@code frame}'s step, each sphere blended {@code alpha} of the way from where the step before it left the
     * sphere to where it did. Waits for the draw, as {@code renderInto} does, and for nothing else: the step's
     * timeline has already reached the value the draw waits for.
     */
    public void show(GuiApp app, ShownRing.Frame frame, float alpha) {
        BoundStorageBuffer slot = bind(app, frame);
        ShownRing.Generation g = frame.generation();
        double[] extent = {g.extent(0), g.extent(1), g.extent(2)};
        double longest = Math.max(extent[0], Math.max(extent[1], extent[2]));
        double scale = longest / 2;                       // metres per orbit unit
        Orbit.Pose pose = orbit.pose();
        double[] eye = {extent[0] / 2 + pose.x() * scale, extent[1] / 2 + pose.y() * scale,
                extent[2] / 2 + pose.z() * scale};
        double[] forward = normalise(-pose.x(), -pose.y(), -pose.z());
        // SdfComposer's camera, which Orbit's angles are written for: right is (cos yaw, 0, −sin yaw), so +x is to the
        // right looking down +z. The other sign mirrors the picture, and a drag then seems to turn the wrong way.
        double[] right = normalise(forward[2], 0, -forward[0]);
        double[] up = cross(forward, right);
        byte[] push = SphereShader.push(eye, right, up, forward, 1.0, alpha, g.spheres(), extent);
        target.renderInto(pipeline, 0L, slot.descriptorSet(), 3, push, 0f, 0f, 0f, 1f, g.timeline(), frame.step());
    }

    /** Points the node at this view's picture; nothing until the first {@link #show} has made one. */
    public void present() {
        if (target != null) {
            node.image(target);
        }
    }

    /**
     * The binding of the frame's slot, made the first time it is drawn. A new generation drops the old one's bindings:
     * its buffers are about to be freed, and a descriptor set may outlive its buffer only if it is never used again.
     */
    private BoundStorageBuffer bind(GuiApp app, ShownRing.Frame frame) {
        if (target == null) {
            target = app.viewport(pixels, pixels);
            node.image(target);
        }
        if (frame.generation().id() != generation) {
            dropSlots();
            generation = frame.generation().id();
        }
        BoundStorageBuffer slot = slots[frame.slot()];
        if (slot == null) {
            slot = new BoundStorageBuffer(app.gpu().device(), frame.generation().slot(frame.slot()),
                    SphereShader.SHOWN_BINDING);
            slots[frame.slot()] = slot;
        }
        if (pipeline == null) {
            pipeline = target.pipelineFor(Fullscreen.triangleVertexWithUvSpirv(), Fullscreen.ENTRY_POINT,
                    SphereShader.fragmentSpirv(), "main", SphereShader.PUSH_BYTES,
                    new long[] {slot.descriptorSetLayout()});
        }
        return slot;
    }

    private void dropSlots() {
        for (int k = 0; k < slots.length; k++) {
            if (slots[k] != null) {
                slots[k].close();
                slots[k] = null;
            }
        }
    }

    private static double[] normalise(double x, double y, double z) {
        double length = Math.sqrt(x * x + y * y + z * z);
        return new double[] {x / length, y / length, z / length};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    /** Releases the pipeline and the bindings. The target is the application's. */
    @Override
    public void close() {
        if (pipeline != null) {
            pipeline.close();
            pipeline = null;
        }
        dropSlots();
    }
}
