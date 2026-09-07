"""Standard DH link transform.

Reference for the ``dh_revolute`` macro in robot_description: the macro
expands one DH row into a revolute joint (d, theta) plus a fixed joint
(a, alpha), and this function is the same product in one step. Kinematics
runs off the URDF, so nothing here is on the runtime path.
"""

import math

import numpy as np


def dh_transform(theta, d, a, alpha):
    """Standard DH link transform: Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    # fmt: off
    return np.array(
        [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d     ],
            [0.0, 0.0,      0.0,     1.0   ],
        ]
    )
    # fmt: on
