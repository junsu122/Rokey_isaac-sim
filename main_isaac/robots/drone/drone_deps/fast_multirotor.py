"""
fast_multirotor.py
Optimized Multirotor subclass for smooth real-time simulation.

Eliminates ~7,500 unnecessary DC interface calls/sec (at 500 Hz physics):
  - No handle_propeller_visual(): removes 9 DC calls/step (4,500/sec)
  - Cached rigid body handles: removes 6 get_rigid_body calls/step (3,000/sec)
"""

import carb
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig


class FastMultirotor(Multirotor):
    """Drop-in Multirotor replacement — no rotor animation, cached body handles."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._rb_rotors = None
        self._rb_body   = None

    def stop(self):
        """Invalidate cached handles on sim stop so they are rebuilt on next play."""
        self._rb_rotors = None
        self._rb_body   = None

    def _cache_bodies(self):
        dc = self.get_dc_interface()
        self._rb_rotors = [
            dc.get_rigid_body(self._stage_prefix + "/rotor" + str(i))
            for i in range(4)
        ]
        self._rb_body = dc.get_rigid_body(self._stage_prefix + "/body")

    def update(self, dt: float):
        # Build cache once per simulation session
        if self._rb_rotors is None:
            self._cache_bodies()

        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0] * self._thrusters._num_rotors

        self._thrusters.set_input_reference(desired_rotor_velocities)
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)

        dc    = self.get_dc_interface()
        zero3 = carb._carb.Float3([0.0, 0.0, 0.0])

        # Apply thrust force on each rotor body (no propeller visual — saves 8 DC calls/step)
        for i in range(4):
            dc.apply_body_force(
                self._rb_rotors[i],
                carb._carb.Float3([0.0, 0.0, float(forces_z[i])]),
                zero3, False,
            )

        # Rolling moment on main body
        dc.apply_body_torque(
            self._rb_body,
            carb._carb.Float3([0.0, 0.0, float(rolling_moment)]),
            False,
        )

        # Linear drag on main body
        drag = self._drag.update(self._state, dt)
        dc.apply_body_force(
            self._rb_body,
            carb._carb.Float3(list(drag)),
            zero3, False,
        )

        for backend in self._backends:
            backend.update(dt)
