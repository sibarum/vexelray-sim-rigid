package dev.vexelray.sim.rigid.demo;

import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.layout.LayoutEnums.AlignItems;
import dev.vexelray.gui.core.layout.LayoutEnums.Direction;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.core.layout.Rect;
import dev.vexelray.gui.core.style.Role;
import dev.vexelray.gui.widget.TitleBar;
import dev.vexelray.sim.core.gui.DemoLook;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.sim.core.gui.OrbitControl;
import dev.vexelray.sim.rigid.gui.SphereView;
import sibarum.tactroller.api.Key;

/**
 * The tree: the picture on the left, the panel on the right, and the keys. Needs the {@code Gui} and no window, so it
 * is built before there is one; the picture's target is made the first time it is drawn.
 */
final class Ui implements AutoCloseable {

    /** The side of the square target the spheres are traced into. */
    private static final int VIEW_PIXELS = 768;

    /** The most of the body's width the picture takes; the panel has the rest, and grows into anything left over. */
    private static final float VIEW_MAX_SHARE = 0.64f;

    private final Node canvas;
    private final Node body;
    private final SphereView view;
    private final OrbitControl orbitControl;
    private final Readings readings;
    private final Panel panel;
    private float side = -1f;

    Ui(Gui gui, TitleBar titleBar, Controls controls, Orbit orbit) {
        canvas = gui.box()
                .corner(DemoLook.CORNER)
                .clip(true)
                .background(gui.theme().color(Role.WELL));
        gui.landmark("view", canvas);
        view = new SphereView(canvas, VIEW_PIXELS, orbit);
        orbitControl = new OrbitControl(gui, canvas, orbit);
        readings = new Readings(gui);
        panel = new Panel(gui, controls, readings);

        body = gui.row()
                .width(Length.FILL).height(Length.grow(1f))
                .gap(DemoLook.GAP).padding(DemoLook.WIDE, DemoLook.WIDE)
                .alignItems(AlignItems.STRETCH)
                .children(canvas, panel.node());
        gui.onResize(body, layout -> fit());
        gui.root().direction(Direction.COLUMN)
                .background(gui.theme().color(Role.PAGE))
                .children(titleBar.node(), body);
        keys(gui, controls, orbit);
    }

    SphereView view() {
        return view;
    }

    Readings readings() {
        return readings;
    }

    Panel panel() {
        return panel;
    }

    /**
     * Makes the picture the largest square the body allows, up to {@link #VIEW_MAX_SHARE} of its width; the panel grows
     * into the rest. The square is a percentage of the body's content box in each axis, so it is exact at any zoom;
     * nothing it sets moves the body, so it cannot feed itself.
     */
    private void fit() {
        Rect room = body.layout().content();
        if (room.w() <= 0f || room.h() <= 0f) {
            return;
        }
        float s = Math.max(1f, Math.min(room.h(), VIEW_MAX_SHARE * room.w()));
        if (side > 0f && Math.abs(s - side) < 0.5f) {
            return;
        }
        side = s;
        canvas.width(Length.percent(100f * s / room.w())).height(Length.percent(100f * s / room.h()));
    }

    /** Every key only records a request; see {@link Controls}. */
    private static void keys(Gui gui, Controls controls, Orbit orbit) {
        gui.shortcut(Key.SPACE, controls::togglePause);
        gui.shortcut(Key.R, controls::reset);
        gui.shortcut(Key.N, controls::nextScenario);
        gui.shortcut(Key.EQUAL, controls::faster);
        gui.shortcut(Key.MINUS, controls::slower);
        gui.shortcut(Key.H, orbit::home);
    }

    /**
     * The orbit's input. Not the view: its pipeline and bindings are made on the window's device the first time it
     * draws, and this part is built before the device exists, so it is closed after the device is. {@link Session},
     * which draws it and is built after the device, closes it.
     */
    @Override
    public void close() {
        orbitControl.close();
    }
}
