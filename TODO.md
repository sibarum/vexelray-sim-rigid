# vexelray-sim-rigid — TODO

## "Runs on" does not show a fallback

When the backend picked in **Runs on** is not available, `Physics.build` falls back (to this GPU, or, with no compute
queue of its own, to the CPU) and says so in the readings, but the dropdown still shows the backend that was asked
for. On llvmpipe it reads *This GPU* while the physics runs on the CPU. Make the dropdown show what is actually
running, or mark the unavailable entries, so the control and the readings agree.

## The picture's pipeline is compiled on the drawing thread

`RigidView.show` builds the picture's graphics pipeline the first time a frame needs it, on the main thread: the
fragment shader's SPIR-V, generated in Java (about 260 ms the first time on the VM, 11 ms after), and the driver's
compile of it (most of the rest; on llvmpipe the first frame held the main thread for about 890 ms in all). Both can
happen elsewhere: generate the SPIR-V on a lane as soon as the shape is known (it is pure), and create the pipeline
off the main thread once `ViewportTarget.pipelineFor` is known to be safe there (`vkCreateGraphicsPipelines` is; what
the target holds may not be), drawing nothing new until it is ready. Real GPUs feel it less, but every one pays it.
