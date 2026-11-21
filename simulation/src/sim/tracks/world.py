"""MuJoCo world for a Track: the Dreamer kart (kart.xml, not modified) plus the track.

build(track) loads kart.xml into an MjSpec and adds, in code:
  - grass ground, an asphalt surface mesh, concrete runoff where a section asks for it
  - painted edge lines, red/white kerbs at corners, a chequered start/finish line
  - edge barriers per section: cones, tyre stacks, concrete or plastic barrier blocks
  - obstacles (free bodies, or mocap bodies that slide across the track)
  - a sky, a sun that follows the kart (so shadows stay sharp near it), render settings
and puts the kart's rear axle on the start pose.

Everything the kart can hit (barriers, cones, obstacles) uses contype 1 / conaffinity 1,
like the floor. Painted lines, kerbs and the surface meshes are visual only.
Call update(model, data) every step to move sliding obstacles.
"""
import math
import pathlib

import mujoco
import numpy as np

from . import meshes
from .format import Track, yaw_quat

KART = pathlib.Path(__file__).resolve().parents[3] / "models" / "dreamer" / "kart.xml"

# Heights of the visual layers. The collision floor stays at z = 0 (hidden); the grass
# you see is a separate plane 2 cm lower, so the asphalt never flickers against it.
Z_GRASS, Z_RUNOFF, Z_ASPHALT, Z_PAINT, Z_KERB = -0.02, 0.0, 0.0, 0.004, 0.006
LINE_WIDTH = 0.10
KERB_IN, KERB_OUT = 0.25, 0.75          # kerb overlap onto the track / width beyond the edge
RUNOFF_WIDTH = 2.5
TEX_TILE = 4.0                          # metres per asphalt/concrete texture tile
OFFSET = {"cones": 0.35, "tyres": 1.1, "concrete": 1.2, "plastic": 1.2}   # edge to item centre
CONE_HEIGHT = 0.45
COLLIDE = {"contype": 1, "conaffinity": 1}
NO_COLLIDE = {"contype": 0, "conaffinity": 0}
MOTION = "motion:"                     # custom numeric "motion:<body>" = [x, y, nx, ny, amp, period, phase]


def build(track: Track, kart=KART, free_cones=False):
    """MjSpec of the kart on the track. free_cones: edge cones can be knocked over
    (each is a free body; slower with many cones). Obstacle cones are always free."""
    spec = mujoco.MjSpec.from_file(str(kart))
    w = _World(spec, track)
    w.environment()
    w.surface()
    for sec in track.sections:
        w.section(sec, free_cones)
    w.start_line()
    w.obstacles()
    w.place_kart()
    return spec


def compile(track: Track, kart=KART, free_cones=False):
    """(model, data) ready to step, kart at the start."""
    model = build(track, kart, free_cones).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def update(model, data):
    """Move the sliding obstacles to their position at data.time (call every step)."""
    for i in range(model.nnumeric):
        name = model.numeric(i).name
        if not name.startswith(MOTION):
            continue
        adr = model.numeric_adr[i]
        x, y, nx, ny, amp, period, phase = model.numeric_data[adr:adr + 7]
        u = amp * math.sin(2 * math.pi * data.time / period + phase)
        m = model.body_mocapid[model.body(name[len(MOTION):]).id]
        data.mocap_pos[m, 0] = x + nx * u
        data.mocap_pos[m, 1] = y + ny * u


class _World:
    def __init__(self, spec, track):
        self.spec, self.t = spec, track
        self.wb = spec.worldbody
        self.n_mesh = 0

    # -------------------------------------------------------------- helpers
    def mesh(self, verts, faces, tex=None):
        name = f"trk_mesh{self.n_mesh}"
        self.n_mesh += 1
        m = self.spec.add_mesh(name=name)
        m.uservert = np.asarray(verts, float).ravel().tolist()
        m.userface = np.asarray(faces, int).ravel().tolist()
        if tex is not None:
            m.usertexcoord = np.asarray(tex, float).ravel().tolist()
            m.userfacetexcoord = np.asarray(faces, int).ravel().tolist()
        m.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
        return name

    def texture(self, name, kind, repeat, uniform=False, **mat):
        img = meshes.texture(kind, seed=len(name))
        h, w_, _ = img.shape
        tex = self.spec.add_texture(name=name, type=mujoco.mjtTexture.mjTEXTURE_2D,
                                    width=w_, height=h, nchannel=3)
        tex.data = img.tobytes()
        m = self.spec.add_material(name=name, texrepeat=repeat, texuniform=uniform, **mat)
        m.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
        return name

    def material(self, name, rgba, **kw):
        self.spec.add_material(name=name, rgba=rgba, **kw)
        return name

    def geom(self, parent=None, **kw):
        kw.setdefault("group", 0)
        for k, v in NO_COLLIDE.items():
            kw.setdefault(k, v)
        return (parent or self.wb).add_geom(**kw)

    def range_idx(self, s0, s1, pad=0):
        t = self.t
        i0, i1 = t.index_at(s0), t.index_at(s1)
        if s1 >= t.length - 1e-6:
            i1 = t.n
        return np.arange(i0 - pad, i1 + 1 + pad) % t.n

    def side_room(self, idx, side):
        """Space (m) beyond the edge before the inside of a corner folds over."""
        k = self.t.curvature[idx] * side
        half = (self.t.half_left if side > 0 else self.t.half_right)[idx]
        return np.where(k > 1e-6, 1 / np.maximum(k, 1e-6) - half, 1e3)

    # -------------------------------------------------------------- scene
    def environment(self):
        s, t = self.spec, self.t
        sky = s.add_texture(name="trk_sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                            builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                            rgb1=[0.42, 0.62, 0.88], rgb2=[0.88, 0.92, 0.96], width=512, height=3072)
        del sky
        s.visual.rgba.haze = [0.80, 0.86, 0.92, 1]
        s.visual.headlight.ambient = [0.32, 0.32, 0.32]
        s.visual.headlight.diffuse = [0.18, 0.18, 0.18]
        s.visual.headlight.specular = [0.0, 0.0, 0.0]
        s.visual.global_.offwidth = 1920       # room for camera images later
        s.visual.global_.offheight = 1080
        s.visual.quality.shadowsize = 8192
        s.visual.quality.offsamples = 8
        s.visual.map.shadowclip = 3.0
        s.visual.map.znear = 0.01
        s.visual.map.zfar = 60.0
        x, y, _ = t.start_pose
        s.stat.center = [x, y, 0.3]
        s.stat.extent = 10.0
        for light in self.wb.lights:          # kart.xml's overhead lamp: dim, no shadow
            light.diffuse = [0.15, 0.15, 0.15]
            light.castshadow = False
        for cam in self.wb.cameras:           # kart.xml's world cameras assume the kart at 0
            cam.pos = self._to_start(cam.pos)
            cam.quat = _qmul(yaw_quat(t.start_pose[2]), cam.quat)
        # Sun: directional, follows the kart so its shadow map covers the area near it.
        sun = s.body("chassis").add_light(name="sun", type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                                          mode=mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM,
                                          pos=[0, 0, 15], castshadow=True,
                                          diffuse=[0.75, 0.73, 0.68], specular=[0.2, 0.2, 0.2],
                                          ambient=[0.05, 0.05, 0.06])
        sun.dir = (np.array([-0.35, 0.25, -0.9]) / np.linalg.norm([-0.35, 0.25, -0.9])).tolist()
        lo, hi = t.bounds(RUNOFF_WIDTH + 60)
        grass = self.texture("trk_grass", "grass", [0.08, 0.08], uniform=True,
                             specular=0.0, shininess=0.0, reflectance=0.0)
        centre = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2]
        size = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.1]
        floor = s.geom("floor")                  # kart.xml's floor: still the collision ground
        floor.pos, floor.size, floor.group = centre + [0], size, 3
        self.geom(name="grass", type=mujoco.mjtGeom.mjGEOM_PLANE, pos=centre + [Z_GRASS],
                  size=size, material=grass)
        self.texture("trk_asphalt", "asphalt", [1, 1], specular=0.1, shininess=0.1)
        self.texture("trk_concrete", "concrete", [1, 1], specular=0.05, shininess=0.05)
        self.texture("trk_kerb", "kerb", [1, 1], specular=0.15)
        ck = s.add_texture(name="trk_checker", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.95, 0.95, 0.95], rgb2=[0.05, 0.05, 0.05], width=64, height=64)
        m = s.add_material(name="trk_checker", texrepeat=[1, 1])
        m.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = ck.name
        self.material("trk_paint", [0.93, 0.93, 0.90, 1], specular=0.1)
        self.material("trk_cone", [0.98, 0.38, 0.05, 1], specular=0.3, shininess=0.4)
        self.material("trk_white", [0.95, 0.95, 0.95, 1], specular=0.3)
        self.material("trk_black", [0.05, 0.05, 0.05, 1], specular=0.05)
        self.material("trk_rubber", [0.06, 0.06, 0.065, 1], specular=0.08, shininess=0.1)
        self.material("trk_red", [0.78, 0.08, 0.07, 1], specular=0.2)
        self.material("trk_jersey", [0.70, 0.69, 0.66, 1], specular=0.05)
        self.material("trk_cardboard", [0.62, 0.47, 0.30, 1], specular=0.05)
        self.material("trk_barrel_blue", [0.08, 0.25, 0.62, 1], specular=0.4, shininess=0.5)
        self.material("trk_barrel_orange", [0.95, 0.45, 0.06, 1], specular=0.4, shininess=0.5)
        self.material("trk_invisible", [0.9, 0.3, 0.3, 0.3])
        # Shared item meshes.
        v, f, _ = meshes.lathe([(0.13, 0.025), (0.11, 0.17), (0.085, 0.27), (0.03, CONE_HEIGHT)], 20)
        self.cone_mesh = self.mesh(v, f)
        v, f, _ = meshes.lathe([(0.1095, 0.18), (0.0905, 0.265)], 20)
        v[:, :2] *= 1.03
        self.cone_band = self.mesh(v, f)
        v, f, _ = meshes.tyre_stack()
        self.tyre_mesh = self.mesh(v, f)
        prof = [(-0.30, 0.0), (0.30, 0.0), (0.30, 0.075), (0.18, 0.33), (0.075, 0.81),
                (-0.075, 0.81), (-0.18, 0.33), (-0.30, 0.075)]
        v, f, _ = meshes.extrude(prof, 1.98)
        self.jersey_mesh = self.mesh(v, f)

    def _to_start(self, p):
        x, y, yaw = self.t.start_pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        return [x + c * p[0] - s_ * p[1], y + s_ * p[0] + c * p[1], p[2]]

    def surface(self):
        t = self.t
        uv = lambda P, _: P / TEX_TILE                                    # noqa: E731
        v, f, tx = meshes.ribbon(t.left_edge, t.right_edge, Z_ASPHALT, uv, closed=True)
        self.geom(name="track_surface", type=mujoco.mjtGeom.mjGEOM_MESH,
                  meshname=self.mesh(v, f, tx), material="trk_asphalt")

    def section(self, sec, free_cones):
        t = self.t
        idx = self.range_idx(sec.s0, sec.s1)
        for side, style in ((1, sec.left), (-1, sec.right)):
            edge = t.offset_line(side, np.zeros(t.n))[idx]
            room = self.side_room(idx, side)
            nrm = t.normal[idx] * side
            if sec.runoff == "concrete":
                width = np.clip(room - 0.5, 0.3, RUNOFF_WIDTH)
                outer = edge + nrm * width[:, None]
                a, b = (outer, edge) if side > 0 else (edge, outer)
                v, f, tx = meshes.ribbon(a, b, Z_RUNOFF, lambda P, _: P / TEX_TILE)
                self.geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.mesh(v, f, tx),
                          material="trk_concrete")
            if style in ("lines", "kerbs"):
                inner = edge - nrm * LINE_WIDTH
                a, b = (edge, inner) if side > 0 else (inner, edge)
                v, f, _ = meshes.ribbon(a, b, Z_PAINT)
                self.geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.mesh(v, f),
                          material="trk_paint")
            if style == "kerbs":
                self.kerbs(idx, side, room)
            elif style == "cones":
                for x, y, yaw in self.along(idx, side, OFFSET["cones"], self.cone_spacing(idx)):
                    self.cone([x, y], yaw, free=free_cones)
            elif style == "tyres":
                for x, y, yaw in self.along(idx, side, OFFSET["tyres"], 0.62):
                    self.geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.tyre_mesh,
                              material="trk_rubber", pos=[x, y, 0], quat=yaw_quat(yaw * 7.0))
                    self.geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.30, 0.30, 0],
                              pos=[x, y, 0.30], group=3, material="trk_invisible", **COLLIDE)
            elif style == "concrete":
                for x, y, yaw in self.along(idx, side, OFFSET["concrete"], 2.0):
                    self.geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.jersey_mesh,
                              material="trk_jersey", pos=[x, y, 0], quat=yaw_quat(yaw), **COLLIDE)
            elif style == "plastic":
                for i, (x, y, yaw) in enumerate(self.along(idx, side, OFFSET["plastic"], 1.25)):
                    self.geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.6, 0.22, 0.4],
                              pos=[x, y, 0.4], quat=yaw_quat(yaw),
                              material="trk_red" if i % 2 else "trk_white", **COLLIDE)

    def cone_spacing(self, idx):
        k = np.abs(self.t.curvature[idx]).max() if len(idx) else 0
        return 2.5 if k > 1 / 25 else 4.0

    def along(self, idx, side, offset, spacing):
        """(x, y, yaw) every `spacing` m along the line `offset` m outside the edge."""
        t = self.t
        room = self.side_room(idx, side)
        off = np.minimum(offset, np.maximum(room - 0.6, 0.2))
        P = t.offset_line(side, np.zeros(t.n))[idx] + t.normal[idx] * side * off[:, None]
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(seg)]
        if s[-1] < spacing:
            return []
        out = []
        for q in np.arange(spacing / 2, s[-1] - spacing / 2 + 1e-6, spacing):
            j = min(int(np.searchsorted(s, q)) , len(P) - 1)
            j0 = max(j - 1, 0)
            d = P[j] - P[j0] if j > j0 else P[1] - P[0]
            f = (q - s[j0]) / max(s[j] - s[j0], 1e-9)
            x, y = P[j0] + (P[j] - P[j0]) * f
            out.append((float(x), float(y), math.atan2(d[1], d[0])))
        return out

    def kerbs(self, idx, side, room):
        """Kerbs on corners: the inside through the apex, the outside on the exit."""
        t = self.t
        k = t.curvature[idx] * side
        corner = np.abs(t.curvature[idx]) > 1 / 25
        runs, i = [], 0
        while i < len(idx):
            if corner[i]:
                j = i
                while j < len(idx) and corner[j]:
                    j += 1
                runs.append((i, j))
                i = j
            else:
                i += 1
        for a, b in runs:
            inside = k[(a + b) // 2] > 0
            if inside:
                a2, b2 = a + (b - a) // 5, b - (b - a) // 5
            else:
                a2, b2 = (a + b) // 2, min(b + 10, len(idx))
            if b2 - a2 < 4:
                continue
            sl = idx[a2:b2]
            edge = t.offset_line(side, np.zeros(t.n))[sl]
            nrm = t.normal[sl] * side
            out_w = np.minimum(KERB_OUT, np.maximum(room[a2:b2] - 0.3, 0.1))
            inner, outer = edge - nrm * KERB_IN, edge + nrm * out_w[:, None]
            a_, b_ = (outer, inner) if side > 0 else (inner, outer)
            arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(edge, axis=0), axis=1))]
            tex = lambda P, col: np.column_stack([arc / 2.0, np.full(len(arc), 0.5 * col)])  # noqa: E731
            v, f, tx = meshes.ribbon(a_, b_, Z_KERB, tex)
            self.geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.mesh(v, f, tx),
                      material="trk_kerb")

    def start_line(self):
        t = self.t
        idx = np.array([t.n - 1, 0, 1])
        left, right = t.left_edge[idx], t.right_edge[idx]
        width = float(t.width[0])
        cell = width / max(round(width / 0.25), 1)
        tex = lambda P, col: np.column_stack([np.arange(3) * 0.25 / cell * 0.5,  # noqa: E731
                                              np.full(3, col * width / cell * 0.5)])
        v, f, tx = meshes.ribbon(left, right, Z_PAINT, tex)
        self.geom(name="start_line", type=mujoco.mjtGeom.mjGEOM_MESH,
                  meshname=self.mesh(v, f, tx), material="trk_checker")

    # -------------------------------------------------------------- items
    def cone(self, xy, yaw, free=True, name=None):
        parent = self.wb
        pos = [xy[0], xy[1], 0.0]
        if free:
            parent = self.wb.add_body(name=name or "", pos=pos, quat=yaw_quat(yaw))
            parent.add_freejoint()
            _inertial(parent, 1.0, [0, 0, 0.12], [0.012, 0.012, 0.008])
            pos, q = [0, 0, 0], [1, 0, 0, 0]
        else:
            q = yaw_quat(yaw)
        self.geom(parent, type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.17, 0.17, 0.0125],
                  pos=[pos[0], pos[1], 0.0125], quat=q, material="trk_black", **COLLIDE)
        self.geom(parent, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.cone_mesh,
                  pos=pos, quat=q, material="trk_cone", **COLLIDE)
        self.geom(parent, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.cone_band,
                  pos=pos, quat=q, material="trk_white")

    def obstacles(self):
        for i, o in enumerate(self.t.obstacles):
            name = f"obstacle{i}_{o.kind}"
            if o.kind == "cone" and not o.moving:
                self.cone([o.x, o.y], o.yaw, free=True, name=name)
                continue
            body = self.wb.add_body(name=name, pos=[o.x, o.y, 0.0], quat=yaw_quat(o.yaw))
            if o.moving:
                body.mocap = True
                self.spec.add_numeric(name=MOTION + name, size=7,
                                      data=[o.x, o.y, o.nx, o.ny, o.amplitude, o.period, o.phase])
            else:
                body.add_freejoint()
            if o.kind == "cone":
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.17, 0.17, 0.0125],
                          pos=[0, 0, 0.0125], material="trk_black", **COLLIDE)
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.cone_mesh,
                          material="trk_cone", **COLLIDE)
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=self.cone_band,
                          material="trk_white")
                mass, ipos, diag = 1.0, [0, 0, 0.12], [0.012, 0.012, 0.008]
            elif o.kind == "box":
                hx, hy, hz = o.size
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, hz], pos=[0, 0, hz],
                          material="trk_cardboard", **COLLIDE)
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx * 1.002, 0.03, hz * 1.002],
                          pos=[0, 0, hz], material="trk_white")              # packing tape
                mass = 4.0
                ipos = [0, 0, hz]
                diag = [mass / 3 * (hy**2 + hz**2), mass / 3 * (hx**2 + hz**2), mass / 3 * (hx**2 + hy**2)]
            else:
                r, hz = o.size
                mat = "trk_barrel_blue" if (i % 2 == 0) else "trk_barrel_orange"
                self.geom(body, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r, hz, 0],
                          pos=[0, 0, hz], material=mat, **COLLIDE)
                for zr in (0.3, 0.7):                                       # rolling hoops
                    self.geom(body, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[r * 1.02, 0.015, 0],
                              pos=[0, 0, 2 * hz * zr], material="trk_black")
                mass = 15.0
                ipos = [0, 0, hz]
                diag = [mass * (3 * r**2 + 4 * hz**2) / 12] * 2 + [mass * r**2 / 2]
            if not o.moving:
                _inertial(body, mass, ipos, diag)

    def place_kart(self):
        x, y, yaw = self.t.start_pose
        ch = self.spec.body("chassis")
        ch.pos = [x, y, ch.pos[2]]
        ch.quat = yaw_quat(yaw)


def _inertial(body, mass, ipos, diag):
    body.explicitinertial = True
    body.mass = mass
    body.ipos = ipos
    body.inertia = diag


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]
