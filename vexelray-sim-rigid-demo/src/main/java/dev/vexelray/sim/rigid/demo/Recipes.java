package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.Configuration;
import dev.vexelray.framework.api.MainThread;
import dev.vexelray.framework.api.Provides;
import dev.vexelray.framework.shell.Appearance;
import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.krono.KronoGui;
import dev.vexelray.gui.widget.TitleBar;
import dev.vexelray.sim.core.gui.DemoLook;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.sim.rigid.gui.ShownRing;
import sibarum.atchung.Atchung;

/**
 * What the demo builds, one recipe a part, and nothing about when: {@code RigidDemoWiring} is generated from this and
 * from {@link Physics}, and builds each part in the phase its parameters put it in.
 *
 * <p>Two parts are shared between the frame and the physics lane, and are the only two: the {@link ShownRing} the
 * finished steps go through, and the {@link PhysicsNews} the readings do. Everything else crosses as a message.
 */
@Configuration
final class Recipes {

    @Provides
    Appearance look() {
        return Appearance.of(DemoLook.THEME, Length.em(60), Length.em(36));
    }

    /** What the user has asked for, held until the next frame takes it. */
    @Provides
    Controls controls() {
        return new Controls();
    }

    @Provides
    Orbit orbit() {
        return new Orbit();
    }

    @Provides
    ShownRing ring() {
        return new ShownRing();
    }

    @Provides
    PhysicsNews news() {
        return new PhysicsNews();
    }

    @Provides
    Ui ui(Gui gui, TitleBar titleBar, Controls controls, Orbit orbit) {
        return new Ui(gui, titleBar, controls, orbit);
    }

    /** The frame's side: needs the window's device to draw with, so it is the main thread's. */
    @Provides
    @MainThread
    Session session(GuiApp app, Ui ui, Controls controls, ShownRing ring, PhysicsNews news, Atchung bus,
                    KronoGui krono) {
        return new Session(app, ui, controls, ring, news, bus, krono);
    }
}
