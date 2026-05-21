import time
import numpy as np

from pathlib import Path
from typing import Union, Tuple
from scipy.spatial.transform import Rotation

from figs.control.base_controller import BaseController
from figs.dynamics.model_specifications import generate_specifications


class VelocityController(BaseController):
    """
    Inner-loop velocity tracking controller.

    Converts desired velocity commands [vx, vy, vz, psi_dot] to body-rate
    commands [uf, wx, wy, wz] compatible with the FiGS ACADOS integrator.

    Uses two cascaded P-controllers:
      - Outer loop: velocity error -> desired acceleration -> required thrust vector
      - Inner loop: attitude error (body z vs desired thrust direction) -> body rates

    Velocity commands arrive via the `obj` argument to control():
        obj = np.array([vx_des, vy_des, vz_des, psi_dot_des])

    Coordinate convention (matches model_equations.py):
        - World frame: z points DOWN (NED-like), gravity = +9.81 in z
        - uf in [-1, 0]  (negative thrust)
        - body rates in [-5, 5] rad/s
    """

    def __init__(self,
                 hz: int = 20,
                 Kv: float = 2.0,
                 Ka: float = 5.0,
                 frame_name: str = "carl",
                 configs_path: Path = None) -> None:
        """
        Args:
            hz:           Control frequency (Hz).
            Kv:           Velocity proportional gain.
            Ka:           Attitude proportional gain. Should satisfy Ka >> Kv
                          for inner-loop bandwidth separation.
            frame_name:   Drone frame config name (loads m and tn).
            configs_path: Path to the configs directory (auto-resolved if None).
        """
        super().__init__(configs_path)

        self.hz = hz
        self.nzcr = None
        self.Kv = Kv
        self.Ka = Ka

        frame_cfg = self.load_json_config("frame", frame_name)
        drn_spec = generate_specifications(frame_cfg)
        self.m = drn_spec["m"]
        self.tn = drn_spec["tn"]

        # Gravity vector in world frame (z-down / NED convention)
        self._g = np.array([0., 0., 9.81])

    def control(self,
                tcr: float,
                xcr: np.ndarray,
                upr: Union[None, np.ndarray],
                obj: Union[None, np.ndarray],
                icr: Union[None, np.ndarray],
                zcr: Union[None, object]
                ) -> Tuple[np.ndarray, None, np.ndarray, np.ndarray]:
        """
        Args:
            tcr: Current time (unused, kept for interface compatibility).
            xcr: State [px, py, pz, vx, vy, vz, qx, qy, qz, qw].
            upr: Previous control (unused).
            obj: Velocity command [vx_des, vy_des, vz_des, psi_dot_des].
                 Defaults to hover [0, 0, 0, 0] if None.
            icr: Image (unused by this controller).
            zcr: Recurrent state (passed through unchanged, always None).

        Returns:
            ucr:  Control [uf, wx, wy, wz].
            zcr:  None.
            adv:  Zeros (4,).
            tsol: Timing [0, solve_time, 0, 0].
        """
        t0 = time.time()

        v_curr = xcr[3:6]
        q_curr = xcr[6:10]  # [qx, qy, qz, qw] scalar-last (Hamilton)

        if obj is not None and len(obj) >= 4:
            v_des = np.asarray(obj[0:3], dtype=float)
            psi_dot_des = float(obj[3])
        else:
            v_des = np.zeros(3)
            psi_dot_des = 0.0

        # ── Outer loop: velocity error → desired acceleration ──────────────
        a_des = self.Kv * (v_des - v_curr)

        # Required thrust force in world frame.
        # From dynamics: v_dot = g + (tn*uf/m)*R[:,2]
        # We need: (tn*uf/m)*R[:,2] = a_des - g  =>  F_req = a_des - g
        F_req = a_des - self._g      # e.g. [0, 0, -9.81] for hover
        F_mag = np.linalg.norm(F_req)

        # Normalized thrust command (clipped to valid range)
        uf = np.clip(-F_mag * self.m / self.tn, -1.0, 0.0)

        # ── Desired body z-axis in world frame ─────────────────────────────
        # Since uf < 0: (tn*uf/m)*z_body = F_req  =>  z_body = -F_req/F_mag
        if F_mag > 1e-6:
            z_des = -F_req / F_mag   # = [0, 0, 1] at hover ✓
        else:
            z_des = np.array([0., 0., 1.])

        # ── Inner loop: attitude error → body rates ─────────────────────────
        R = Rotation.from_quat(q_curr).as_matrix()  # body-to-world (3×3)
        z_curr = R[:, 2]

        # Axis of rotation to align z_curr → z_des, expressed in world frame.
        e_world = np.cross(z_curr, z_des)
        # Convert to body frame for body-rate commands.
        e_body = R.T @ e_world

        omega_x = np.clip(self.Ka * e_body[0], -5.0, 5.0)
        omega_y = np.clip(self.Ka * e_body[1], -5.0, 5.0)
        omega_z = np.clip(psi_dot_des, -5.0, 5.0)

        ucr = np.array([uf, omega_x, omega_y, omega_z])
        tsol = np.array([0., time.time() - t0, 0., 0.])
        adv = np.zeros(4)

        return ucr, zcr, adv, tsol
