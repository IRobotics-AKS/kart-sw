"""Small mesh and texture builders for the track world (no image files needed)."""
import numpy as np


def ribbon(left, right, z, uv=None, closed=False):
    """Flat strip between two (N, 2) polylines at height z, facing up.

    left must be on the left of the direction of travel. uv(points) -> (N, 2) texture
    coordinates for the left and right points; default is none.
    Returns (vertices (M, 3), faces (K, 3), texcoords (M, 2) or None).
    """
    n = len(left)
    verts = np.zeros((2 * n, 3))
    verts[0::2, :2], verts[1::2, :2] = left, right
    verts[:, 2] = z
    m = n if closed else n - 1
    a = 2 * np.arange(m)
    b = 2 * ((np.arange(m) + 1) % n)
    faces = np.concatenate([np.column_stack([a, a + 1, b]), np.column_stack([a + 1, b + 1, b])])
    tex = None
    if uv is not None:
        tex = np.zeros((2 * n, 2))
        tex[0::2], tex[1::2] = uv(left, 0), uv(right, 1)
    return verts, faces, tex


def lathe(profile, segments=24, closed=False):
    """Surface of revolution about z. profile: [(r, z), ...] from bottom to top.

    closed=False: open profile, capped at the first and last point (cones, barrels).
    closed=True: the profile is a loop (a tyre's cross-section).
    """
    prof = np.asarray(profile, float)
    k = len(prof)
    th = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    verts = [[r * np.cos(t), r * np.sin(t), z] for r, z in prof for t in th]
    faces = []
    rows = k if closed else k - 1
    for i in range(rows):
        i2 = (i + 1) % k
        for j in range(segments):
            j2 = (j + 1) % segments
            a, b, c, d = i * segments + j, i * segments + j2, i2 * segments + j2, i2 * segments + j
            faces += [[a, b, c], [a, c, d]]
    if not closed:
        bottom, top = len(verts), len(verts) + 1
        verts += [[0, 0, prof[0, 1]], [0, 0, prof[-1, 1]]]
        last = (k - 1) * segments
        for j in range(segments):
            j2 = (j + 1) % segments
            faces += [[bottom, j2, j], [top, last + j, last + j2]]
    return np.array(verts), np.array(faces), None


def extrude(profile, length):
    """Prism: a closed (y, z) profile extruded along x, centred on x = 0."""
    prof = np.asarray(profile, float)
    k = len(prof)
    verts = [[x, y, z] for x in (-length / 2, length / 2) for y, z in prof]
    faces = []
    for i in range(k):
        i2 = (i + 1) % k
        faces += [[i, i2, k + i2], [i, k + i2, k + i]]
    for i in range(1, k - 1):
        faces += [[0, i + 1, i], [k, k + i, k + i + 1]]
    return np.array(verts), np.array(faces), None


def tyre_stack(count=3, r_in=0.17, r_out=0.30, height=0.20, segments=20):
    """`count` kart-circuit tyres stacked (one visual mesh)."""
    t = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    rm, rc, hc = (r_in + r_out) / 2, (r_out - r_in) / 2, height / 2
    c, s = np.cos(t), np.sin(t)
    ring = np.column_stack([rm + rc * np.sign(c) * np.abs(c) ** 0.5,                 # squarish section
                            hc + 0.95 * hc * np.sign(s) * np.abs(s) ** 0.5])
    V, F = [], []
    for i in range(count):
        v, f, _ = lathe([(r, z + i * height) for r, z in ring], segments, closed=True)
        F.append(f + sum(len(x) for x in V))
        V.append(v)
    return np.vstack(V), np.vstack(F), None


# ------------------------------------------------------------------ textures (RGB bytes)
def _noise(rng, size, scale):
    """Smooth value noise in [0, 1] at roughly `scale` pixels per blob (tileable)."""
    n = max(2, size // scale)
    g = rng.random((n, n))
    x = np.arange(size) * n / size
    i0 = np.floor(x).astype(int)
    f = x - i0
    f = f * f * (3 - 2 * f)
    i1 = (i0 + 1) % n
    rows = g[i0] * (1 - f)[:, None] + g[i1] * f[:, None]
    return rows[:, i0] * (1 - f)[None] + rows[:, i1] * f[None]


def texture(kind, size=1024, seed=0):
    rng = np.random.default_rng(seed)
    fine = rng.random((size, size))
    if kind == "asphalt":
        base = 0.20 + 0.05 * _noise(rng, size, 64) + 0.07 * (fine - 0.5)
        speck = (rng.random((size, size)) > 0.985) * 0.12
        img = np.stack([base + speck] * 3, axis=-1) * [1.0, 1.0, 1.04]
    elif kind == "grass":
        patches = 0.6 * _noise(rng, size, 256) + 0.4 * _noise(rng, size, 48)
        blades = fine - 0.5
        img = np.stack([0.20 + 0.06 * patches + 0.05 * blades,
                        0.36 + 0.10 * patches + 0.08 * blades,
                        0.12 + 0.03 * patches + 0.03 * blades], axis=-1)
    elif kind == "concrete":
        base = 0.55 + 0.06 * _noise(rng, size, 80) + 0.05 * (fine - 0.5)
        img = np.stack([base, base * 0.99, base * 0.96], axis=-1)
    elif kind == "kerb":                      # two stripes along u: red then white
        img = np.ones((size // 8, size, 3)) * [0.92, 0.92, 0.92]
        img[:, : size // 2] = [0.80, 0.10, 0.08]
        img = img + 0.04 * (rng.random(img.shape[:2]) - 0.5)[..., None]
    else:
        raise ValueError(kind)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)
