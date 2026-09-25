# Cruise control, phase C

This project studies control around a cruise operating point using an **existing XSWIFT aircraft simulation environment**. I did not build that environment or its underlying aircraft model. My work here was to design and assess control laws and integrate a digital controller with the available simulation.

## Process and decisions

1. **Work from the supplied trim and linear models.** [`XSWIFT_h=100_v=30.mat`](XSWIFT_h%3D100_v%3D30.mat) provides the operating-point data used by the design scripts. The simulation initialization sets 100 m altitude and 30 m/s airspeed.
2. **Design longitudinal control.** [`longitudinal.m`](longitudinal.m) adds altitude to the longitudinal state, designs an LQR feedback law, and includes an angle-of-attack observer. It examines initial angle-of-attack and speed perturbations and the control demand induced by wind, with actuator bandwidth in view.
3. **Design lateral control.** [`latero_direccional_place.m`](latero_direccional_place.m) examines aircraft modes and controllability, then explores pole placement with heading and cross-track states while checking normalized actuator limits. [`latero_direccional_LQR.m`](Simulation/latero_direccional_LQR.m) is an alternative LQR design that evaluates an initial-condition case and a gust case.
4. **Integrate and inspect.** [`implementacion_digital_controles.m`](Simulation/implementacion_digital_controles.m) implements the lateral and longitudinal control calculations used with [`XSWIFT_cruise.slx`](Simulation/XSWIFT_cruise.slx). [`Results`](Simulation/Results/) contains saved simulation cases, and [`Plotter_resultados.m`](Simulation/Results/Plotter_resultados.m) plots commands, flight states, altitude error, cross-track error, and wind.

## Where to look

- [`Simulation/XSWIFT_cruise.slx`](Simulation/XSWIFT_cruise.slx): current cruise simulation, based on the existing environment.
- [`longitudinal.m`](longitudinal.m) and [`latero_direccional_place.m`](latero_direccional_place.m): controller-design analyses.
- [`Simulation/Results`](Simulation/Results/): recorded simulation cases and plotting script.
- [`References`](References/): supporting diagrams.

The Simulink model changes MATLAB's current folder to `Simulation`, runs `XSWIFT_cruise_init.m`, and loads the local `long_cruise.mat` and `lat_cruise.mat` controller data. Run the top-level design scripts from the `Cruise` folder; the lateral LQR script in `Simulation` also needs `Cruise` on the MATLAB path to find the trim data.
