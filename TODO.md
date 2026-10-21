# vexelray-sim-rigid — TODO

## "Runs on" does not show a fallback

When the backend picked in **Runs on** is not available, `Physics.build` falls back (to this GPU, or, with no compute
queue of its own, to the CPU) and says so in the readings, but the dropdown still shows the backend that was asked
for. On llvmpipe it reads *This GPU* while the physics runs on the CPU. Make the dropdown show what is actually
running, or mark the unavailable entries, so the control and the readings agree.

## CPU fallback stalls the drawing thread

With no compute queue of its own, every step's copy into the picture is handed to the main thread
(`ComputeQueue.drawingQueue`). On llvmpipe that showed as 0.5–0.9 s stalls there, mostly at setup (the keep kernel's
compile in `ShownSlots`) and then in each copy waiting for its turn between frames. Only the queue submissions need
the main thread: look at building `ShownSlots` (allocations, kernel registration) on the physics lane and handing
over only the submits.
