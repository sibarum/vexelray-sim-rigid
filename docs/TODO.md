# TODO

- [ ] **A first experiment: boxes and spheres that fall and stack.** Contact by impulses solved one after another, or
      by XPBD (extended position-based dynamics), or both, each measured on the same scenes: a stack's drift and
      jitter at rest, momentum and energy across a collision, and cost per step.
- [ ] **Fixed point against f32**, as the fluid's scatter was measured: whether an exact integer state buys
      repeatability (replays, lockstep) worth its range.
- [ ] **The budgeted clock** (`dev.vexelray.sim.core.time.Clock`) driving the step, as it drives the shallow-water
      patch.
- [ ] **Contact against static SDF geometry**, the same field the fluid meets.
- [ ] **Rest costs nothing**: bodies that have settled sleep.
- [ ] **A demo**, once there is something to show.
