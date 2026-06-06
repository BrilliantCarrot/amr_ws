"""
Isaac Sim dynamic obstacle proxy for W5 LiDAR/CBF avoidance test.

Usage in Isaac Sim:
  1) Open the warehouse USD scene.
  2) Press Play once so simulation/update callbacks run.
  3) Open Window > Script Editor.
  4) Run:
       exec(open('/path/to/isaac_dynamic_obstacle_proxy.py').read())

The script creates a visible cylinder obstacle and moves it left-right across
Carter's path. It is intended as a moving pedestrian proxy for LiDAR-based
obstacle detection and CBF/FrontSafety avoidance validation.
"""

from pxr import UsdGeom, UsdPhysics, Gf, Sdf
import omni.usd
import omni.timeline
import omni.kit.app
import math

# -----------------------------
# User-tunable parameters
# -----------------------------
PRIM_PATH = "/World/dynamic_obstacle_proxy"

# Motion line center. For a straight robot path along +X, place the obstacle
# around x=1.5~2.0 and sweep in Y so it crosses the robot's route.
CENTER_X = 2.0
CENTER_Y = -1.0
BASE_Z = 0.0

# Cylinder dimensions. Height is large enough for LiDAR/camera visibility.
RADIUS = 0.22
HEIGHT = 1.50

# Sweeping motion: y = CENTER_Y + AMPLITUDE_Y * sin(...)
AMPLITUDE_Y = 0.45
PERIOD_SEC = 20.0

# Set True to print pose periodically in Isaac Sim console.
VERBOSE = True
PRINT_INTERVAL_SEC = 1.0

# -----------------------------
# Internal state
# -----------------------------
_stage = omni.usd.get_context().get_stage()
_timeline = omni.timeline.get_timeline_interface()
_app = omni.kit.app.get_app()

_state = {
    "sub": None,
    "t": 0.0,
    "last_print": -999.0,
}


def _ensure_cylinder():
    """Create or reuse a cylinder prim and add collision metadata."""
    prim = _stage.GetPrimAtPath(PRIM_PATH)

    if not prim or not prim.IsValid():
        cyl = UsdGeom.Cylinder.Define(_stage, Sdf.Path(PRIM_PATH))
        prim = cyl.GetPrim()
        cyl.CreateRadiusAttr(RADIUS)
        cyl.CreateHeightAttr(HEIGHT)
        cyl.CreateAxisAttr("Z")

        # Give it a simple display color so it is easy to see in the viewport.
        cyl.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.2, 0.1)])
    else:
        cyl = UsdGeom.Cylinder(prim)
        if cyl:
            cyl.GetRadiusAttr().Set(RADIUS)
            cyl.GetHeightAttr().Set(HEIGHT)
            cyl.GetAxisAttr().Set("Z")

    # Add collision API. RTX LiDAR generally sees renderable geometry, but
    # collision also makes the proxy physically meaningful if later needed.
    try:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
    except Exception as exc:
        print(f"[DynamicObstacleProxy] CollisionAPI warning: {exc}")

    # Initial transform. Cylinder is centered at z=BASE_Z + HEIGHT/2.
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(CENTER_X, CENTER_Y, BASE_Z + HEIGHT * 0.5))

    return prim, translate_op


_prim, _translate_op = _ensure_cylinder()


def _on_update(event):
    """Move the obstacle once per Isaac Sim update tick."""
    # Use wall/sim update dt from event payload when available.
    dt = 1.0 / 60.0
    try:
        if event.payload and "dt" in event.payload:
            dt = float(event.payload["dt"])
    except Exception:
        pass

    # Only move while timeline is playing. This makes pause/resume intuitive.
    if not _timeline.is_playing():
        return

    _state["t"] += dt

    omega = 2.0 * math.pi / max(PERIOD_SEC, 1e-6)
    y = CENTER_Y + AMPLITUDE_Y * math.sin(omega * _state["t"])
    x = CENTER_X
    z = BASE_Z + HEIGHT * 0.5

    _translate_op.Set(Gf.Vec3d(x, y, z))

    if VERBOSE and (_state["t"] - _state["last_print"] >= PRINT_INTERVAL_SEC):
        _state["last_print"] = _state["t"]
        print(
            f"[DynamicObstacleProxy] path={PRIM_PATH} "
            f"pos=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"radius={RADIUS:.2f} height={HEIGHT:.2f}"
        )


# Remove previous subscription if the script was run before in the same session.
_old = globals().get("_DYNAMIC_OBSTACLE_PROXY_SUB", None)
if _old is not None:
    try:
        _old.unsubscribe()
    except Exception:
        pass

_sub = _app.get_update_event_stream().create_subscription_to_pop(
    _on_update,
    name="dynamic_obstacle_proxy_update",
)
_state["sub"] = _sub
_DYNAMIC_OBSTACLE_PROXY_SUB = _sub

print("[DynamicObstacleProxy] Started.")
print(f"[DynamicObstacleProxy] Created/moving: {PRIM_PATH}")
print("[DynamicObstacleProxy] Press Play in Isaac Sim. Re-run this script to reset/update parameters.")
