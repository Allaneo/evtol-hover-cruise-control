# eVTOL flight control: hover and cruise

I developed two related MATLAB/Simulink control projects during my aerospace engineering studies. They address different flight regimes and have different starting points.

| Project | My contribution | Starting environment |
| --- | --- | --- |
| [Hover, phases A and B](Hover/README.md) | Built the hover simulation environment and designed its nested motor, attitude, velocity, and position control loops. | My Simulink model. |
| [Cruise, phase C](Cruise/README.md) | Designed, integrated, and assessed longitudinal and lateral control laws. | An existing XSWIFT cruise simulation environment. I did **not** build its aircraft simulation environment. |

## Engineering approach

For hover, I worked from the inner motor loop outward. I examined frequency response and stability margins, then designed the slower vehicle loops around response and actuator limits. The [Phase B presentation](Hover/Phase%20B/FASE%20B.pptx) records a useful design correction: an initial position-control target was too oscillatory, prompting a revised, more practical response shape.

For cruise, I started from a provided aircraft model and trim data. I developed longitudinal and lateral feedback designs, considered disturbances and control demand, and integrated digital control into the XSWIFT simulation. The [cruise project](Cruise/README.md) identifies the scripts and saved simulation cases.

These files document simulation and controller-design work. They are not evidence of flight testing or a certified flight-control system. Recorded simulation cases are included; a clean-environment reproduction log is not yet available.
