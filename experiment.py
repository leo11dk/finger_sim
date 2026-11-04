"""Physical parameter controls and measurements for the two-joint finger."""
import math
import mujoco
import numpy as np

LIMITS = {"torque": (0., .06), "k1": (100., 2000.), "k2": (100., 2000.),
          "r1": (12., 16.), "r2": (12., 16.)}


class FingerExperiment:
    def __init__(self, model, data):
        self.model, self.data = model, data
        self.act = model.actuator("reel_torque_Nm").id
        self.reel_joint = model.joint("reel_joint").id
        self.reel_body = model.body("reel").id
        self.coupling = model.equality("reel_coupling").id
        self.reel_radius = float(model.geom_size[model.geom("reel_core").id, 0])
        self.flex = model.tendon("flexor").id
        self.springs = [model.tendon(f"return_spring_{i}").id for i in (1, 2)]
        self.guides = [model.geom(f"guide_{i}").id for i in (1, 2)]
        self.joints = [model.joint(f"joint_{i}").id for i in (1, 2)]
        self.defaults = dict(torque=0., k1=float(model.tendon_stiffness[self.springs[0]]),
                             k2=float(model.tendon_stiffness[self.springs[1]]),
                             r1=float(model.geom_size[self.guides[0], 0]*1000),
                             r2=float(model.geom_size[self.guides[1], 0]*1000))
        self.params = self.defaults.copy()
        self.parameter_controls = {
            "k1": model.actuator("spring_1_N_per_m").id,
            "k2": model.actuator("spring_2_N_per_m").id,
            "r1": model.actuator("pulley_1_radius_mm").id,
            "r2": model.actuator("pulley_2_radius_mm").id,
        }
        self._write_controls()
        self.reference_length = self._straight_length()
        mujoco.mj_forward(model, data)

    def _write_controls(self):
        self.data.ctrl[self.act] = self.params["torque"]
        for key, actuator in self.parameter_controls.items():
            self.data.ctrl[actuator] = self.params[key]

    def sync_controls(self):
        """Read native parameter sliders before stepping the physics."""
        lo, hi = LIMITS["torque"]
        self.params["torque"] = float(np.clip(self.data.ctrl[self.act], lo, hi))
        updates = {}
        for key, actuator in self.parameter_controls.items():
            raw = float(self.data.ctrl[actuator])
            # Native Reset/Clear all zeros ctrl. Retain mechanical parameters.
            value = self.params[key] if raw == 0. else float(np.clip(raw, *LIMITS[key]))
            if value != self.params[key]:
                updates[key] = value
        if updates:
            self.apply(updates)
        else:
            self._write_controls()

    def _straight_length(self):
        reference = mujoco.MjData(self.model)
        reference.qpos[:] = self.model.qpos0
        mujoco.mj_forward(self.model, reference)
        return float(reference.ten_length[self.flex])

    def apply(self, updates):
        unknown = set(updates) - set(LIMITS)
        if unknown:
            raise ValueError(f"Unknown parameters: {sorted(unknown)}")
        values = {key: float(value) for key, value in updates.items()}
        for key, value in values.items():
            low, high = LIMITS[key]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be in [{low}, {high}]")
        new = self.params | values
        changed_radius = any(new[key] != self.params[key] for key in ("r1", "r2"))
        for i, key in enumerate(("k1", "k2")):
            self.model.tendon_stiffness[self.springs[i]] = new[key]
        if changed_radius:
            for i, key in enumerate(("r1", "r2")):
                gid = self.guides[i]
                self.model.geom_size[gid, 0] = new[key] / 1000.
                radius, half_length = self.model.geom_size[gid, :2]
                self.model.geom_rbound[gid] = math.hypot(radius, half_length)
            # Recompute configuration-dependent constants using scratch state.
            mujoco.mj_setConst(self.model, mujoco.MjData(self.model))
            self.reference_length = self._straight_length()
        self.params = new
        self._write_controls()
        mujoco.mj_forward(self.model, self.data)

    def reset(self, defaults=False):
        self.apply(self.defaults if defaults else {"torque": 0.})
        mujoco.mj_resetData(self.model, self.data)
        self._write_controls()
        mujoco.mj_forward(self.model, self.data)

    def measure(self):
        # Keep all reported kinematics at the current integrated state.
        mujoco.mj_forward(self.model, self.data)
        angles = [float(np.rad2deg(self.data.qpos[self.model.jnt_qposadr[j]]))
                  for j in self.joints]
        extension = [max(0., float(self.data.ten_length[t]
                         - self.model.tendon_lengthspring[t, 1])) for t in self.springs]
        rows = np.flatnonzero((self.data.efc_type[:self.data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                              & (self.data.efc_id[:self.data.nefc] == self.coupling))
        # Signed equality reaction; positive means tensile cable force.
        cable_force = -float(self.data.efc_force[rows[0]]) if len(rows) else 0.
        reel_angle = float(self.data.qpos[self.model.jnt_qposadr[self.reel_joint]])
        return dict(reel_angle_deg=float(np.rad2deg(reel_angle)),
                    reel_take_up_mm=self.reel_radius*reel_angle*1000,
                    cable_force=cable_force,
                    torque=float(self.data.ctrl[self.act]),
                    params=self.params.copy(), time=float(self.data.time),
                    angles=angles,
                    shortening_mm=(self.reference_length-float(self.data.ten_length[self.flex]))*1000,
                    spring_extension_mm=[x*1000 for x in extension],
                    spring_elastic_force=[float(self.model.tendon_stiffness[t])*x
                                          for t, x in zip(self.springs, extension)],
                    warnings=[int(w.number) for w in self.data.warning])
