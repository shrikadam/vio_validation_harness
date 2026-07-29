### Situation
"Recently, I led an optimization effort for MegaPose—a state-of-the-art monovular RGB 6D object pose estimation pipeline—to get it running on edge GPU hardware. Out of the box, the model was mathematically accurate, but it chugged along at a sluggish 7 FPS. For any real-time spatial awareness or interactive control loop, that level of latency is an immediate dealbreaker."
### Task
"My goal was to break through the 25 FPS real-time threshold and cut latency down to sub-35 milliseconds, all while operating within a strict 2.8GB VRAM footprint."
### Action
"Instead of making assumptions, I profiled the execution stack using PyTorch Profiler and Chrome Tracing. The data revealed a massive bottleneck: the GPU was sitting idle because a desktop game engine (Panda3D) was serializing render templates through CPU multiprocessing queues, bottlenecking the PCIe bus.
To fix this, I re-architected the execution pipeline:
First, I eliminated the game engine and replaced it with Nvidia nvdiffrast, building a zero-copy C++ CUDA rasterization bridge that kept CAD geometry and render outputs completely inside VRAM.
Second, when pose tracking initially hallucinated due to coordinate system conflicts, I derived a custom $4 \times 4$ projection matrix from scratch to cleanly map OpenCV camera space to OpenGL NDC space.
Finally, I enforced strict FP32 boundary guards to prevent PyTorch’s mixed-precision engine from passing fragmented memory into bare-metal C++ kernels."
### Result
"The results were immediate: frame rates jumped from 7 FPS to over 27 FPS—a nearly 4x speedup—while VRAM usage dropped comfortably under 2.8GB with rock-solid pose accuracy."
### Value
"What this experience reinforced for me is that delivering high-fidelity perception on constrained edge devices isn't just about tweaking neural net weights—it requires diving all the way down to memory layouts, matrix transformations, and GPU silicon. I bring this exact ability to profile, debug, and optimize complex spatial vision algorithms across the entire software and hardware stack."