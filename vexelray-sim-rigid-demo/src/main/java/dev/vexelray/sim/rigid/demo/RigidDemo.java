package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.shell.VexelApplication;

/**
 * The rigid experiments, on screen: a scenario's spheres, the solver's settings, and the readings they move.
 *
 * <p>The entry point and its constants, and nothing else, as the project builder's template has it: what the
 * application builds is in {@link Recipes}, one recipe a part, and the physics is {@link Physics}, a component on a
 * lane of its own. {@code RigidDemoAppWiring}, which builds them in order, is generated from those and from the
 * {@code @VexelApp} on {@code RigidDemoApp}.
 *
 * <pre>
 * RigidDemo                    the window
 * RigidDemo --automation=0     the window, driveable on a free port (ottermate reads it off stdout)
 * </pre>
 *
 * <p>The {@code @VexelApp} declaration is {@code RigidDemoApp}; this class holds the facts it reads.
 *
 * Needs {@code --enable-native-access=ALL-UNNAMED}; {@code mvn exec:exec} passes it.
 */
public final class RigidDemo {

    /** The settings directory's name. Stable across releases, because changing it forgets window placement. */
    static final String APP = "vexelray-sim-rigid-demo";

    static final String TITLE = "Rigid experiments";

    /** First-run window size, in logical coordinates. */
    static final int W = 1280;
    static final int H = 820;

    private RigidDemo() {
    }

    public static void main(String[] args) {
        VexelApplication.run(new RigidDemoAppWiring(), args);
    }
}
