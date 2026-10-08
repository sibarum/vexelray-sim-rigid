package dev.vexelray.sim.rigid.sphere;

/**
 * The broad phase's grid: {@code nx × ny × nz} cubic cells of side {@code cell}, from the origin. A sphere is
 * sorted into the cell its centre is in, and tests the spheres of that cell and the 26 around it — every sphere it
 * can touch, provided no sphere is wider than a cell. That is the one condition, and {@link #covering} holds the
 * cell to it; a sphere past the grid is still found, counted into the nearest cell.
 *
 * <p>Cell {@code (ix, iy, iz)} is key {@code ix + nx · (iy + ny · iz)}.
 */
public record SphereGrid(int nx, int ny, int nz, float cell) {

    public SphereGrid {
        if (nx < 1 || ny < 1 || nz < 1 || !(cell > 0)) {
            throw new IllegalArgumentException("a grid of at least one cell, of positive size, got " + nx + " × "
                    + ny + " × " + nz + " of " + cell);
        }
    }

    /** The grid over a box from the origin to {@code (sx, sy, sz)}, its cells as small as spheres this wide allow. */
    public static SphereGrid covering(double sx, double sy, double sz, double widest) {
        if (!(widest > 0)) {
            throw new IllegalArgumentException("the widest sphere must have a positive diameter, got " + widest);
        }
        return new SphereGrid(cells(sx, widest), cells(sy, widest), cells(sz, widest), (float) widest);
    }

    /** How many cells, and so keys, there are. */
    public int cells() {
        return Math.multiplyExact(Math.multiplyExact(nx, ny), nz);
    }

    private static int cells(double extent, double cell) {
        return Math.max(1, (int) Math.ceil(extent / cell));
    }
}
