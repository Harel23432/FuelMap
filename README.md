This project simulates how an engine control unit (ECU) determines injector pulse widths from real-time engine state, capturing core control-system concepts like fuel maps, interpolation, cold-start enrichment, and closed-loop O₂ feedback under embedded-style constraints.
I designed the system to resemble embedded control software rather than a desktop simulation:
Fuel delivery is computed through 2D lookup tables with bilinear interpolation, matching real fuel map behavior.
A custom fixed-size slab allocator models environments where dynamic allocation is constrained or forbidden.
Cold-start enrichment and closed-loop correction are separated into explicit stages to mirror real control pipelines.
A controller interface abstracts system state from control logic, allowing other systems to plug into the same framework later.
The program is a standalone C++ executable. Running it initializes a fuel map, injector model, and controller, then computes injector pulse width for a sample engine state. Parameters like RPM, load, coolant temperature, and AFR can be modified in main() to simulate different operating conditions.
The allocator debug output shows remaining free blocks to demonstrate memory behavior.
I intentionally traded physical accuracy and real-time constraints for code clarity and architectural realism. The simulation does not model combustion physics, timing constraints, or actual sensor noise. Closed-loop correction uses a simplified linear model rather than a PID controller to keep behavior explainable and predictable.
The allocator is included for demonstration and reasoning, not because this system requires custom allocation for correctness.
