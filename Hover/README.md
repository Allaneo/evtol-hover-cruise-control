# Hover control, phases A and B

This project develops a hover simulation environment and a set of nested controllers for an eVTOL vehicle. I built the Simulink environment for this part of the work. The engineering question was how to coordinate fast motor and attitude responses with slower velocity and position tracking while respecting response and disturbance-rejection requirements.

## Process and decisions

1. **Start with the motor loops.** [`LI_final.m`](LI_final.m) examines the inner loop with Bode, Nyquist, root-locus, and sensitivity plots. [`LE_final.m`](LE_final.m) designs the outer motor-speed loop and records the motor and propeller parameters used in the model. Some values in that script are explicitly estimates rather than measurements.
2. **Build the vehicle-level cascade.** The scripts in [`Phase B`](Phase%20B/) design velocity, position, pitch-angle, and angular-speed controllers. [`eVTOL.slx`](eVTOL.slx) brings these controller data into the hover simulation.
3. **Revise the position-control target.** The [Phase B presentation](Phase%20B/FASE%20B.pptx) shows that an initial ITAE-based position design led to an overly oscillatory response. It then explores Bessel and affine-parameterization alternatives, including the effect of controller saturation. This is a design trade-off, not a claim that every requirement was met.
4. **Compare continuous and sampled implementations.** [`validacion_modelos.m`](Phase%20B/validacion_modelos.m) compares the analog inner-loop design with a Tustin-discretized controller and a zero-order-hold plant using frequency-response plots.

## Where to look

- [`eVTOL.slx`](eVTOL.slx): current hover model.
- [`LI_final.m`](LI_final.m), [`LE_final.m`](LE_final.m), and [`Phase B`](Phase%20B/): controller-design work.
- [`FASE B.pptx`](Phase%20B/FASE%20B.pptx): requirements, alternatives, and design discussion.
- [`Examples`](Examples/): a separate 3GL example model, not the main hover model.

To inspect the current model in MATLAB, set the current folder to `Hover` before opening `eVTOL.slx`. Its initialization callback loads the controller `.mat` files beside the model. Run the design scripts from this folder if you want their saved `.mat` outputs to be written where the model expects them.
