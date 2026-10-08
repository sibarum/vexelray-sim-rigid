package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.Fullscreen;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.vulkan.present.BoundStorageBuffer;
import dev.vexelray.vulkan.present.GraphicsPipeline;
import dev.vexelray.vulkan.present.SampledColorTarget;

/**
 * Spheres on screen, ray-traced straight from the simulation's own buffer into a target a node shows.
 *
 * <p>The state is never copied to the host for this: the view binds {@link SphereSimulation#shownBuffer} and reads it
 * where the last step left it, so the cost of the picture is one fullscreen pass. That works only because the
 * simulation computes on the device the application draws on.
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
    private BoundStorageBuffer shown;
    private long boundBuffer;

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
     * Draws {@code sim} as it stands, each sphere blended {@code alpha} of the way from where the step before the last
     * left it to where the last one did.
     *
     * @param extent the box, {@code sx, sy, sz}, in metres
     */
    public void show(GuiApp app, SphereSimulation sim, float alpha, double[] extent) {
        bind(app, sim);
        // The steps are finished before the draw is submitted: that wait is the dependency between the kernels that
        // wrote the buffer and the shader that reads it.
        sim.finish();

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
        byte[] push = SphereShader.push(eye, right, up, forward, 1.0, alpha, sim.spheres(), extent);
        target.renderInto(pipeline, 0L, shown.descriptorSet(), 3, push, 0f, 0f, 0f, 1f);
    }

    /** Points the node at this view's picture; nothing until the first {@link #show} has made one. */
    public void present() {
        if (target != null) {
            node.image(target);
        }
    }

    private void bind(GuiApp app, SphereSimulation sim) {
        if (target == null) {
            target = app.viewport(pixels, pixels);
            node.image(target);
        }
        long buffer = sim.shownBuffer();
        if (buffer == boundBuffer && pipeline != null) {
            return;
        }
        if (shown != null) {
            shown.close();
        }
        shown = new BoundStorageBuffer(app.gpu().device(), buffer, SphereShader.SHOWN_BINDING);
        boundBuffer = buffer;
        if (pipeline == null) {
            pipeline = target.pipelineFor(Fullscreen.triangleVertexWithUvSpirv(), Fullscreen.ENTRY_POINT,
                    SphereShader.fragmentSpirv(), "main", SphereShader.PUSH_BYTES,
                    new long[] {shown.descriptorSetLayout()});
        }
    }

    private static double[] normalise(double x, double y, double z) {
        double length = Math.sqrt(x * x + y * y + z * z);
        return new double[] {x / length, y / length, z / length};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    /** Releases the pipeline and the binding. The target is the application's. */
    @Override
    public void close() {
        if (pipeline != null) {
            pipeline.close();
            pipeline = null;
        }
        if (shown != null) {
            shown.close();
            shown = null;
        }
    }
}
