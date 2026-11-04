"""Visual-only extension-spring coils; physics lives in finger.xml."""
import mujoco
import numpy as np

SPRINGS = [("spring_1_anchor", "spring_1_tip"),
           ("spring_2_anchor", "spring_2_tip")]


def draw_springs(model, data, scene):
    """Append coils to an mjv scene using the physical spring's anchor sites."""
    for first, second in SPRINGS:
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)
               for n in (first, second)]
        if min(ids) < 0:
            continue
        a, b = data.site_xpos[ids[0]], data.site_xpos[ids[1]]
        axis = b - a
        length = np.linalg.norm(axis)
        if length < 1e-8:
            continue
        axis = axis / length
        helper = np.array([0., 1., 0.])
        if abs(axis @ helper) > 0.9:
            helper = np.array([0., 0., 1.])
        u = np.cross(axis, helper)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        points = [a.copy()]
        for t in np.linspace(0, 1, 73):
            # Short straight attachment legs flank six active visual turns.
            envelope = min(1., t * 12., (1. - t) * 12.)
            radial = np.cos(12*np.pi*t)*u + np.sin(12*np.pi*t)*v
            points.append(a + (0.12 + 0.76*t)*length*axis
                          + 0.0025*envelope*radial)
        points.append(b.copy())
        for start, end in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                               np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                               np.array([0.15, 0.45, 1., 1.]))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                0.00045, start, end)
            scene.ngeom += 1


def draw_reel_cables(model, data, scene):
    """Show cable winding on the single groove; physics uses the reel equality."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "reel")
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "reel_joint")
    if bid < 0 or jid < 0:
        return
    center = data.xpos[bid]
    # Base is fixed; cable exits at the bottom of the constant-radius drum.
    radius = model.geom_size[model.geom("reel_core").id, 0]
    angle = data.qpos[model.jnt_qposadr[jid]]
    wrap = 3*np.pi + max(0., angle)
    for y in (0.,):
        phi = np.linspace(-np.pi/2-wrap, -np.pi/2, 120)
        points = center + np.column_stack((radius*np.cos(phi),
                                          np.full_like(phi, y),
                                          radius*np.sin(phi)))
        for a, b in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom:
                return
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                               np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                               np.array([.9, .15, .12, 1.]))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, .0007, a, b)
            scene.ngeom += 1
