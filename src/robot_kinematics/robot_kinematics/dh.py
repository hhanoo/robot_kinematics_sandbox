"""UR10e standard DH parameters (must stay in sync with
src/robot_description/urdf/ur10e.urdf.xacro)."""

import math

import numpy as np

# Standard DH rows: (a, d, alpha). theta comes from the joint angle.
# fmt: off
UR10E_DH = np.array(
    [
        [0.0,      0.1807,   math.pi / 2],   # joint 1
        [-0.6127,  0.0,      0.0        ],   # joint 2
        [-0.57155, 0.0,      0.0        ],   # joint 3
        [0.0,      0.17415,  math.pi / 2],   # joint 4
        [0.0,      0.11985, -math.pi / 2],   # joint 5
        [0.0,      0.11655,  0.0        ],   # joint 6
    ]
)
# fmt: on


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
