package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.FrameStage;
import dev.vexelray.framework.automation.Driver;
import dev.vexelray.framework.shell.AppInfo;
import dev.vexelray.framework.shell.Appearance;
import dev.vexelray.framework.shell.Shell;
import dev.vexelray.framework.shell.Wiring;
import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.layout.LayoutEnums.AlignItems;
import dev.vexelray.gui.core.layout.LayoutEnums.Direction;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.core.layout.Rect;
import dev.vexelray.gui.core.style.Role;
import dev.vexelray.sim.core.gui.DemoLook;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.sim.core.gui.OrbitControl;
import dev.vexelray.sim.rigid.gui.SphereView;
import sibarum.tactroller.api.Key;

import java.util.Set;

/**
 * What the demo builds, and in which phase: the controls are state, so {@code MODEL}; the tree and the keys need only
 * the {@code Gui}, so {@code TREE}; the simulation needs the window's device and the clock, so {@code ATTACH}.
 */
final class RigidDemoWiring extends Wiring {

    private static final AppInfo INFO = new AppInfo(RigidDemo.APP, RigidDemo.TITLE, RigidDemo.W, RigidDemo.H, Set.of());

    /** The side of the square target the spheres are traced into. */
    private static final int VIEW_PIXELS = 768;

    /** The most of the body's width the picture takes; the panel has the rest, and grows into anything left over. */
    private static final float VIEW_MAX_SHARE = 0.64f;

    private final Controls controls = new Controls();
    private final Orbit orbit = new Orbit();
    private SphereView view;
    private Readings readings;
    private Panel panel;
    private Node canvas;
    private Node body;
    private float side = -1f;

    @Override
    public AppInfo info() {
        return INFO;
    }

    @Override
    public void config(Shell shell) {
        shell.appearance(Appearance.of(DemoLook.THEME, Length.em(60), Length.em(36)));
    }

    @Override
    public void tree(Shell shell) {
        Gui gui = shell.gui();
        canvas = gui.box()
                .corner(DemoLook.CORNER)
                .clip(true)
                .background(gui.theme().color(Role.WELL));
        gui.landmark("view", canvas);
        view = new SphereView(canvas, VIEW_PIXELS, orbit);
        shell.disposer().register(new OrbitControl(gui, canvas, orbit));
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
                .children(shell.titleBar().node(), body);
        keys(gui);
    }

    @Override
    public void attach(Shell shell) {
        Session session = shell.disposer().register(new Session(shell, controls, view, readings));
        shell.disposer().register(view);
        shell.hooks().add(FrameStage.APP, panel::sync);
        shell.hooks().add(FrameStage.APP, session::frame);
        // The driving socket, off unless --automation asks for it.
        shell.disposer().register(Driver.open(shell));
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
    private void keys(Gui gui) {
        gui.shortcut(Key.SPACE, controls::togglePause);
        gui.shortcut(Key.R, controls::reset);
        gui.shortcut(Key.N, controls::nextScenario);
        gui.shortcut(Key.EQUAL, controls::faster);
        gui.shortcut(Key.MINUS, controls::slower);
        gui.shortcut(Key.H, orbit::home);
    }
}
