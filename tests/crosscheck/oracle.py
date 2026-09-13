#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BIOBUZZ Shot Sim - independent reference oracle
================================================

A from-scratch Python re-implementation of SPEC.md section 3 (HIVE geometry),
section 4 (ball flight) and section 5 (hit test), with the section 10 decisions,
used to cross-check src/engine.js. It was written from SPEC.md and data/field.json
only; it shares no code with the JavaScript engine and deliberately uses a finer,
table-free method:

  * every shot is integrated on its own (no trajectory table, no 5 ms samples):
    RK4 on the in-plane state (rho, z, v_rho, v_z) with dt = 0.5 ms;
  * collisions are checked at the end of every RK4 step and at cubic-Hermite
    sub-steps in between, so the ball centre moves at most 0.2 r between two
    checks, everywhere on the field (no HIVE bounding-box shortcut);
  * the pentagon, solid-prism, open-CELL-shell, capsule and panel distances are
    computed from first principles, with all geometry read from data/field.json.

Obstacles (SPEC 5 + SPEC 10 defaults)
    target HIVE: up-CELL = zero-thickness shell open at aOut, back wall at aIn;
                 down-CELL = solid prism
    other HIVE:  both CELLs = solid prisms
    both HIVEs:  arm bar (hive.armBar polyline + radius, body plane w = 0)
    frame:       4 legs + crossbar (frame.tubeRadius), 2 cornerBlocks (their own radius)
    not obstacles unless opted in: frame.footBars, frame.logoPanels; FLOWERs never.
    floor: z < r.  walls: |x| or |y| > field.half - r with z < field.wallHeight + r.
    left the field: |x| or |y| > field.half + 6.  can't score: descending,
    z < hive.lipZ - r and the centre not inside the target CELL.

Usage
-----
    python tests/crosscheck/oracle.py <cases.json> <out.json> [options]
    python tests/crosscheck/oracle.py --selftest

cases.json : list of cases (or {"cases": [...]})
    {"id": any,
     "ballId": "pollen" | "nectar",
     "target": "red" | "blue",
     "hiveState": {"red": -1|1, "blue": -1|1},   # upSide sigma per HIVE (default: match start)
     "robot": {"x": in, "y": in},                 # shooter exit point = robot centre (not clamped)
     "h0": in,                                    # exit height above the tile surface
     "S0": float,                                 # launch spin ratio
     "shots": [{"thetaDeg": deg, "v": m/s, "yawDeg": deg}, ...],
     "yawMode": "absolute" | "offset",            # optional, default absolute (or --yaw-offset)
     "options": {...}                             # optional per-case toggles, see below
    }
    thetaDeg = pitch above horizontal. v = exit speed.
    yawDeg   = ABSOLUTE heading psi in the field frame (degrees from +x, counter-clockwise).
               With yawMode "offset" it is an offset from psi0, the heading from the robot
               to the target mouth centroid (SPEC 7.1).

out.json : [{"id": ..., "results": [{"hit", "cause", "apexIn", "tToCell"}, ...]}, ...]
    hit      true only for a clean score (SPEC 5): the centre crosses the mouth plane
             f = a - aOut = 0 inward with (w, b) inside the pentagon and reaches f < -r
             with no contact anywhere up to and including that sample.
    cause    null for a hit, else short | long | lip | roof | cell | frame | wall | floor
             ("timeout" is reserved for a shot still undecided after --tmax; never seen).
             short/long (can't score / left the field): the centre's distance along the
             heading is less / more than the mouth centroid's projection on the heading.
             lip/roof: target-shell contact with the CENTRE's body b < b0 + 1.5 at the
             contact instant -> lip, else roof.
    apexIn   apex of the free flight (obstacles ignored), world z of the ball centre, inches.
    tToCell  hits only: time (s) at which the ball centre crossed the mouth plane (f = 0)
             moving inward inside the pentagon; null for misses.
    With --extra each result also carries diagnostics:
      event              score | contact | floor | wall | out | cantScore | truncated | timeout
      obstacle           name of the first obstacle touched (contact events)
      tEvent             time of the deciding event (contact: interpolated first-touch
                         instant; score: first check with f < -r)
      eventXYZ, eventB   centre position and target-body b at that instant
      contactB           b of the nearest target-shell point at first touch (shell contacts)
      causeContactPoint  lip/roof re-classified with contactB < b0 + 1.5 (alternative reading)
      apexToEventIn      highest centre z up to the event
      minClearanceIn     smallest (distance - r) to any obstacle along the checked path; a
                         result with |minClearanceIn| of a few hundredths is borderline
      entrySpeedMps, entryWB   speed and (w, b) at the mouth-plane crossing (hits)
      sMouthIn           mouth-centroid projection on the heading (the short/long split)
      psiDeg, psi0Deg    absolute heading used, heading to the mouth centroid

Options (CLI flags; the same keys may be given per case in "options"):
    --logo-panels        logo panels are obstacles                  (logoPanels, default off)
    --foot-bars          frame.footBars are obstacles               (footBars, default off)
    --no-corner-blocks   ignore frame.cornerBlocks                  (cornerBlocks, default on)
    --half X             wall inner face |x|,|y| (default field.half = 70.5)      (half)
    --wall-height X      wall height above the tiles (default field.wallHeight)   (wallHeight)
    --out-margin X       "left the field" beyond half + X (default 6)             (outMargin)
    --table-limits       emulate the SPEC 4 table truncation: a shot still        (tableLimits,
                         undecided at t > 2.6 s or rho > 5.6 m is a short/long     default off)
                         miss (event "truncated")
    --yaw-offset         default yawMode "offset" for cases that do not set it
    --dt S               RK4 step, seconds (default 0.0005)
    --step-frac F        max centre travel between checks, in radii (default 0.2)
    --tmax S             safety stop (default 6 s)
    --extra              add diagnostic fields to every result
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
FIELD_JSON = os.path.join(ROOT, 'data', 'field.json')

# ----------------------------------------------------------------------------------
# SPEC 4 constants (SI unless named _in)
# ----------------------------------------------------------------------------------
M_PER_IN = 0.0254
RHO_AIR = 1.20
G = 9.81
CD = 0.45
CL_PER_S = 0.20          # C_L(S) = 0.20 * min(S, 1)

BALLS = {
    'pollen': {'D_in': 2.80, 'm_kg': 24.948e-3},
    'nectar': {'D_in': 91.948 / 25.4, 'm_kg': 41.277e-3},
}

DT = 0.5e-3
STEP_FRAC = 0.2
TMAX = 6.0
LIP_BAND = 1.5           # SPEC 5: shell contact with centre b < b0 + 1.5 -> "lip"

TABLE_T_MAX = 2.6       # SPEC 4 table truncation, emulated only with --table-limits
TABLE_RHO_MAX_M = 5.6

CAUSES = [None, 'short', 'long', 'lip', 'roof', 'cell', 'frame', 'wall', 'floor', 'timeout']
(C_NONE, C_SHORT, C_LONG, C_LIP, C_ROOF, C_CELL, C_FRAME, C_WALL, C_FLOOR,
 C_TIMEOUT) = range(len(CAUSES))
EVENTS = ['score', 'contact', 'floor', 'wall', 'out', 'cantScore', 'timeout', 'truncated']
(EV_SCORE, EV_CONTACT, EV_FLOOR, EV_WALL, EV_OUT, EV_CANT, EV_TIMEOUT, EV_TRUNC) = range(len(EVENTS))
KIND_SHELL, KIND_CELL, KIND_FRAME = 0, 1, 2


def load_field(path=FIELD_JSON):
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def ball_coeffs(ball_id, cd=CD, cl_per_s=CL_PER_S):
    """Return (r_in, kd, klc). kd = 1/2 rho C_D A / m ; lift = klc*min(w r, |v|)*(-vz, vr)."""
    b = BALLS[ball_id]
    r_in = b['D_in'] / 2.0
    r_m = r_in * M_PER_IN
    area = math.pi * r_m * r_m
    kd = 0.5 * RHO_AIR * cd * area / b['m_kg']
    klc = 0.5 * RHO_AIR * cl_per_s * area / b['m_kg']
    return r_in, kd, klc


# ----------------------------------------------------------------------------------
# SPEC 3 geometry + SPEC 5 distance functions
# ----------------------------------------------------------------------------------
class Geometry:
    """HIVE / frame / wall geometry from data/field.json. HIVE index 0 = red, 1 = blue."""

    def __init__(self, field, logoPanels=False, cornerBlocks=True, footBars=False,
                 half=None, wallHeight=None, outMargin=6.0):
        # SPEC 10 defaults: legs, crossbar, cornerBlocks and both arm bars are obstacles;
        # footBars and logoPanels are drawn only (opt in with footBars=True / logoPanels=True).
        hv = field['hive']
        self.pz = float(hv['pivotZ'])
        ang = math.radians(float(hv['armDeg']))
        self.ca, self.sa = math.cos(ang), math.sin(ang)
        self.hx = np.array([float(hv['hiveX']['red']), float(hv['hiveX']['blue'])])
        self.up_start = {'red': int(hv['upSideStart']['red']), 'blue': int(hv['upSideStart']['blue'])}

        cell = hv['cell']
        self.aIn = float(cell['aIn'])
        self.aOut = float(cell['aOut'])
        self.depth = self.aOut - self.aIn
        self.b0 = b0 = float(cell['b0'])
        W, H, SH = float(cell['width']), float(cell['height']), float(cell['shoulder'])
        V = np.array([[-W / 2, b0], [W / 2, b0], [W / 2, b0 + SH], [0.0, b0 + H], [-W / 2, b0 + SH]])
        area2 = np.sum(V[:, 0] * np.roll(V[:, 1], -1) - np.roll(V[:, 0], -1) * V[:, 1])
        if area2 < 0:                                   # force counter-clockwise order
            V = V[::-1].copy()
            area2 = -area2
        self.V = V
        self.E = np.roll(V, -1, axis=0) - V
        self.EL2 = np.sum(self.E * self.E, axis=1)
        # area-weighted polygon centroid (shoelace)
        x0, y0 = V[:, 0], V[:, 1]
        x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
        cr = x0 * y1 - x1 * y0
        self.pent_area = area2 / 2.0
        self.cw = float(np.sum((x0 + x1) * cr) / (3.0 * area2))
        self.cb = float(np.sum((y0 + y1) * cr) / (3.0 * area2))
        self.lipZ = float(hv['lipZ'])

        ab = np.array(hv['armBar']['ab'], float)
        self.arm_P = ab
        self.arm_A = ab[:-1]
        self.arm_D = ab[1:] - ab[:-1]
        self.arm_L2 = np.sum(self.arm_D ** 2, axis=1)
        self.arm_r = float(hv['armBar']['radius'])

        fr = hv['frame']
        caps = []
        for i, (p, q) in enumerate(fr['legs']):
            caps.append((p, q, fr['tubeRadius'], 'leg%d' % i))
        caps.append((fr['crossbar'][0], fr['crossbar'][1], fr['tubeRadius'], 'crossbar'))
        if cornerBlocks and 'cornerBlocks' in fr:
            for i, (p, q) in enumerate(fr['cornerBlocks']['segments']):
                caps.append((p, q, fr['cornerBlocks']['radius'], 'cornerBlock%d' % i))
        if footBars and 'footBars' in fr:
            for i, (p, q) in enumerate(fr['footBars']['segments']):
                caps.append((p, q, fr['footBars']['radius'], 'footBar%d' % i))
        self.cap_A = np.array([c[0] for c in caps], float).reshape(-1, 3)
        self.cap_B = np.array([c[1] for c in caps], float).reshape(-1, 3)
        self.cap_D = self.cap_B - self.cap_A
        self.cap_L2 = np.sum(self.cap_D ** 2, axis=1)
        self.cap_r = np.array([c[2] for c in caps], float)
        cap_names = [c[3] for c in caps]

        self.quads = []
        quad_names = []
        if logoPanels and 'logoPanels' in fr:
            thick = float(fr['logoPanels']['thickness'])
            for i, q in enumerate(fr['logoPanels']['quads']):
                Q = np.array(q, float)
                n = np.cross(Q[1] - Q[0], Q[2] - Q[0])
                n /= np.linalg.norm(n)
                if abs(np.dot(Q[3] - Q[0], n)) > 1e-6:
                    raise ValueError('logo panel %d is not planar' % i)
                QE = np.roll(Q, -1, axis=0) - Q
                M = np.cross(n[None, :], QE)               # in-plane edge normals
                cen = Q.mean(axis=0)
                sgn = np.sign(np.sum((cen - Q) * M, axis=1))
                M = M * sgn[:, None]                       # point inward
                self.quads.append((Q, QE, np.sum(QE * QE, axis=1), n, M, thick / 2.0))
                quad_names.append('logoPanel%d' % i)

        fld = field['field']
        self.half = float(fld['half'] if half is None else half)
        self.wallH = float(fld['wallHeight'] if wallHeight is None else wallHeight)
        self.outLim = self.half + float(outMargin)

        self.names = (['targetUpCell', 'targetDownCell', 'otherUpCell', 'otherDownCell',
                       'armBarRed', 'armBarBlue'] + cap_names + quad_names)
        self.kind = np.array([KIND_SHELL, KIND_CELL, KIND_CELL, KIND_CELL, KIND_FRAME, KIND_FRAME]
                             + [KIND_FRAME] * (len(cap_names) + len(quad_names)), np.int8)
        self.K = len(self.names)

    # ---- frames -------------------------------------------------------------------
    def to_body(self, x, y, z, hive, sigma):
        """World (x, y, z) -> HIVE body (w, a, b) for HIVE index `hive` with upSide sigma."""
        w = x - self.hx[hive]
        u = sigma * y
        dz = z - self.pz
        a = u * self.ca + dz * self.sa
        b = -u * self.sa + dz * self.ca
        return w, a, b

    def to_world(self, w, a, b, hive, sigma):
        u = a * self.ca - b * self.sa
        z = self.pz + a * self.sa + b * self.ca
        return self.hx[hive] + w, sigma * u, z

    def mouth_centroid_world(self, hive, sigma):
        return self.to_world(self.cw, self.aOut, self.cb, hive, sigma)

    def heading_to_mouth(self, x0, y0, hive, sigma):
        """(psi0 rad, horizontal distance in): heading from (x0, y0) to the target mouth centroid."""
        cx, cy, _ = self.to_world(self.cw, self.aOut, self.cb, hive, sigma)
        return np.arctan2(cy - y0, cx - x0), np.hypot(cx - x0, cy - y0)

    # ---- 2-D pentagon ---------------------------------------------------------------
    def pent(self, w, b, nearest=False):
        """(dEdge, inside[, nw, nb]): distance to the pentagon boundary polyline, the
        inside-or-on test (convex, counter-clockwise), and optionally the nearest boundary point."""
        w = np.asarray(w, float)
        b = np.asarray(b, float)
        px = w[..., None] - self.V[:, 0]
        py = b[..., None] - self.V[:, 1]
        ex, ey = self.E[:, 0], self.E[:, 1]
        t = np.clip((px * ex + py * ey) / self.EL2, 0.0, 1.0)
        dx = px - t * ex
        dy = py - t * ey
        d2 = dx * dx + dy * dy
        d_edge = np.sqrt(np.min(d2, axis=-1))
        inside = np.all(ex * py - ey * px >= 0.0, axis=-1)
        if not nearest:
            return d_edge, inside
        j = np.argmin(d2, axis=-1)
        tj = np.take_along_axis(t, j[..., None], axis=-1)[..., 0]
        return d_edge, inside, self.V[j, 0] + tj * ex[j], self.V[j, 1] + tj * ey[j]

    @staticmethod
    def solid_prism(d_edge, inside, a, amin, amax):
        d_poly = np.where(inside, 0.0, d_edge)
        d_int = np.maximum(np.maximum(amin - a, a - amax), 0.0)
        return np.hypot(d_poly, d_int)

    def shell(self, d_edge, inside, a):
        """Target up-CELL: zero-thickness pentagonal tube a in [aIn, aOut], open at aOut and
        closed by a back wall at aIn (the manual stages NECTAR "contacting the back wall").

        SPEC 5 lists d = dEdge for a centre inside the cavity (-depth <= f <= 0). That branch
        leaves out the back wall and is discontinuous at f = -depth, where the SPEC's own
        f < -depth branch (hypot(dPoly, f + depth), i.e. a closed back) gives 0. The exact
        distance there is min(dEdge, f + depth). The two differ only when f + depth < dEdge,
        i.e. f < -(depth - inradius) = -5.7 in, while a shot is decided as a score as soon
        as f < -r (r <= 1.81 in), so hit/miss results are the same either way."""
        f = a - self.aOut
        d_poly = np.where(inside, 0.0, d_edge)
        mid = np.where(inside, np.minimum(d_edge, f + self.depth), d_poly)
        deep = np.hypot(d_poly, f + self.depth)
        return np.where(f > 0.0, np.hypot(d_edge, f), np.where(f >= -self.depth, mid, deep))

    def shell_spec_literal(self, d_edge, inside, a):
        """SPEC 5 text as written (no back wall inside the cavity); used only by the self-test."""
        f = a - self.aOut
        d_poly = np.where(inside, 0.0, d_edge)
        mid = np.where(inside, d_edge, d_poly)
        deep = np.hypot(d_poly, f + self.depth)
        return np.where(f > 0.0, np.hypot(d_edge, f), np.where(f >= -self.depth, mid, deep))

    def shell_nearest(self, w, a, b):
        """Nearest point (w, a, b) on the target shell: rim, side walls or back wall."""
        w = np.asarray(w, float)
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        d_edge, inside, nw, nb = self.pent(w, b, nearest=True)
        f = a - self.aOut
        back = inside & ((f < -self.depth) | ((f <= 0.0) & (f + self.depth < d_edge)))
        na = np.clip(a, self.aIn, self.aOut)
        return (np.where(back, w, nw), np.where(back, self.aIn, na), np.where(back, b, nb))

    def shell_nearest_b(self, w, a, b):
        """b coordinate of the nearest point on the target shell."""
        return self.shell_nearest(w, a, b)[2]

    def armbar(self, w, a, b):
        """Capsule polyline in body coordinates at w = 0 (distance - radius)."""
        pa = a[..., None] - self.arm_A[:, 0]
        pb = b[..., None] - self.arm_A[:, 1]
        da, db = self.arm_D[:, 0], self.arm_D[:, 1]
        t = np.clip((pa * da + pb * db) / self.arm_L2, 0.0, 1.0)
        qa = pa - t * da
        qb = pb - t * db
        d2 = np.min(qa * qa + qb * qb, axis=-1)
        return np.sqrt(w * w + d2) - self.arm_r

    def capsules(self, x, y, z):
        """(N, nCaps) distance to every world capsule surface (distance to segment - radius)."""
        px = x[:, None] - self.cap_A[:, 0]
        py = y[:, None] - self.cap_A[:, 1]
        pz = z[:, None] - self.cap_A[:, 2]
        dx, dy, dz = self.cap_D[:, 0], self.cap_D[:, 1], self.cap_D[:, 2]
        t = np.clip((px * dx + py * dy + pz * dz) / self.cap_L2, 0.0, 1.0)
        qx = px - t * dx
        qy = py - t * dy
        qz = pz - t * dz
        return np.sqrt(qx * qx + qy * qy + qz * qz) - self.cap_r

    @staticmethod
    def _quad_dist(x, y, z, quad):
        Q, QE, QL2, n, M, half_t = quad
        px = x - Q[0, 0]
        py = y - Q[0, 1]
        pz = z - Q[0, 2]
        h = px * n[0] + py * n[1] + pz * n[2]
        inside = np.ones(x.shape, bool)
        dmin2 = np.full(x.shape, np.inf)
        for i in range(4):
            qx = x - Q[i, 0]
            qy = y - Q[i, 1]
            qz = z - Q[i, 2]
            inside &= (qx * M[i, 0] + qy * M[i, 1] + qz * M[i, 2]) >= 0.0
            t = np.clip((qx * QE[i, 0] + qy * QE[i, 1] + qz * QE[i, 2]) / QL2[i], 0.0, 1.0)
            ex = qx - t * QE[i, 0]
            ey = qy - t * QE[i, 1]
            ez = qz - t * QE[i, 2]
            dmin2 = np.minimum(dmin2, ex * ex + ey * ey + ez * ez)
        return np.where(inside, np.abs(h), np.sqrt(dmin2)) - half_t

    def clearances(self, X, Y, Z, sig, T, r):
        """(N, K) clearance (surface distance - r) to every obstacle, plus target body coords.

        sig: (N, 2) upSide per HIVE; T: (N,) target HIVE index; r: (N,) ball radius (in).
        """
        n = X.shape[0]
        per = []
        for k in (0, 1):
            w, a, b = self.to_body(X, Y, Z, k, sig[:, k])
            de, ins = self.pent(w, b)
            per.append(dict(w=w, a=a, b=b, ins=ins,
                            shell=self.shell(de, ins, a),
                            up=self.solid_prism(de, ins, a, self.aIn, self.aOut),
                            down=self.solid_prism(de, ins, a, -self.aOut, -self.aIn),
                            arm=self.armbar(w, a, b)))
        tr = (T == 0)
        R0, R1 = per
        C = np.empty((n, self.K))
        C[:, 0] = np.where(tr, R0['shell'], R1['shell'])
        C[:, 1] = np.where(tr, R0['down'], R1['down'])
        C[:, 2] = np.where(tr, R1['up'], R0['up'])
        C[:, 3] = np.where(tr, R1['down'], R0['down'])
        C[:, 4] = R0['arm']
        C[:, 5] = R1['arm']
        j = 6
        nc = self.cap_A.shape[0]
        if nc:
            C[:, j:j + nc] = self.capsules(X, Y, Z)
            j += nc
        for quad in self.quads:
            C[:, j] = self._quad_dist(X, Y, Z, quad)
            j += 1
        C -= r[:, None]
        wT = np.where(tr, R0['w'], R1['w'])
        aT = np.where(tr, R0['a'], R1['a'])
        bT = np.where(tr, R0['b'], R1['b'])
        insT = np.where(tr, R0['ins'], R1['ins'])
        return C, wT, aT, bT, insT


# ----------------------------------------------------------------------------------
# SPEC 4 flight
# ----------------------------------------------------------------------------------
def accel(vr, vz, kd, klc, wr):
    """a = -kd|v|v + kl|v|(-vz, vr) - g z_hat, kl|v| = klc*min(S,1)*|v| = klc*min(w r, |v|)."""
    s = np.sqrt(vr * vr + vz * vz)
    lift = klc * np.minimum(wr, s)
    return -kd * s * vr - lift * vz, -kd * s * vz + lift * vr - G


def rk4_step(rho, z, vr, vz, kd, klc, wr, dt):
    a1r, a1z = accel(vr, vz, kd, klc, wr)
    v2r, v2z = vr + 0.5 * dt * a1r, vz + 0.5 * dt * a1z
    a2r, a2z = accel(v2r, v2z, kd, klc, wr)
    v3r, v3z = vr + 0.5 * dt * a2r, vz + 0.5 * dt * a2z
    a3r, a3z = accel(v3r, v3z, kd, klc, wr)
    v4r, v4z = vr + dt * a3r, vz + dt * a3z
    a4r, a4z = accel(v4r, v4z, kd, klc, wr)
    rho1 = rho + dt / 6.0 * (vr + 2.0 * v2r + 2.0 * v3r + v4r)
    z1 = z + dt / 6.0 * (vz + 2.0 * v2z + 2.0 * v3z + v4z)
    vr1 = vr + dt / 6.0 * (a1r + 2.0 * a2r + 2.0 * a3r + a4r)
    vz1 = vz + dt / 6.0 * (a1z + 2.0 * a2z + 2.0 * a3z + a4z)
    return rho1, z1, vr1, vz1


# ----------------------------------------------------------------------------------
# SPEC 5 hit test along directly integrated flights (vectorised over shots)
# ----------------------------------------------------------------------------------
def simulate(geom, S, dt=DT, step_frac=STEP_FRAC, tmax=TMAX, table_limits=False):
    """Fly and judge every shot. S holds per-shot arrays:
    r (in), kd, klc, wr (= S0*v0, m/s), v0 (m/s), theta, psi (rad), x0, y0, h0 (in),
    T (target HIVE index), sig (N, 2) upSide per HIVE.
    Returns a dict of per-shot result arrays."""
    N = int(S['r'].shape[0])
    K = geom.K
    nan = np.full(N, np.nan)
    R = dict(hit=np.zeros(N, bool), event=np.full(N, -1, np.int8), cause=np.zeros(N, np.int8),
             obst=np.full(N, -1, np.int16), tEvent=nan.copy(), tCross=nan.copy(),
             apexFree=np.full(N, -np.inf), apexEvent=np.full(N, -np.inf),
             minClear=np.full(N, np.inf), entrySpeed=nan.copy(), entryW=nan.copy(),
             entryB=nan.copy(), evX=nan.copy(), evY=nan.copy(), evZ=nan.copy(), evB=nan.copy(),
             contactB=nan.copy(), sMouth=nan.copy(), psi0=nan.copy())
    if N == 0:
        return R

    cpsi = np.cos(S['psi'])
    spsi = np.sin(S['psi'])
    T = S['T'].astype(np.int64)
    sig = S['sig'].astype(float)
    # aim reference for short/long: mouth centroid of the target up-CELL, projected on the heading
    sigT = sig[np.arange(N), T]
    cx, cy, _ = geom.mouth_centroid_world(T, sigT)
    smouth = (cx - S['x0']) * cpsi + (cy - S['y0']) * spsi
    R['sMouth'][:] = smouth
    R['psi0'][:] = np.arctan2(cy - S['y0'], cx - S['x0'])
    rho_trunc = TABLE_RHO_MAX_M / M_PER_IN

    A = dict(gi=np.arange(N), rho=np.zeros(N), z=np.zeros(N),
             vr=S['v0'] * np.cos(S['theta']), vz=S['v0'] * np.sin(S['theta']),
             r=S['r'].astype(float), kd=S['kd'].astype(float), klc=S['klc'].astype(float),
             wr=S['wr'].astype(float), x0=S['x0'].astype(float), y0=S['y0'].astype(float),
             h0=S['h0'].astype(float), cpsi=cpsi, spsi=spsi, T=T, sig=sig, smouth=smouth,
             fprev=np.full(N, np.nan), wprev=np.full(N, np.nan), bprev=np.full(N, np.nan),
             sprev=np.full(N, np.nan), Cprev=np.full((N, K), np.nan),
             entered=np.zeros(N, bool), apexDone=np.zeros(N, bool), decided=np.zeros(N, bool))

    def check(t, tprev, rho, z, vr, vz, first):
        Zall = A['h0'] + z / M_PER_IN
        nd = ~A['apexDone']
        if nd.any():
            g = A['gi'][nd]
            R['apexFree'][g] = np.maximum(R['apexFree'][g], Zall[nd])
        A['apexDone'] |= (vz < 0.0)
        ui = np.nonzero(~A['decided'])[0]
        if ui.size == 0:
            return
        gi = A['gi'][ui]
        rr = A['r'][ui]
        rho_in = rho[ui] / M_PER_IN
        X = A['x0'][ui] + rho_in * A['cpsi'][ui]
        Y = A['y0'][ui] + rho_in * A['spsi'][ui]
        Z = Zall[ui]
        VZ = vz[ui]
        SP = np.hypot(vr[ui], VZ)
        C, wT, aT, bT, insT = geom.clearances(X, Y, Z, A['sig'][ui], A['T'][ui], rr)
        cmin = C.min(axis=1)
        R['minClear'][gi] = np.minimum(R['minClear'][gi], cmin)
        R['apexEvent'][gi] = np.maximum(R['apexEvent'][gi], Z)
        fT = aT - geom.aOut
        n = ui.size
        ev = np.full(n, -1, np.int8)
        cause = np.zeros(n, np.int8)
        obst = np.full(n, -1, np.int16)
        tev = np.full(n, t)

        # 1. contact with any obstacle (any contact = miss); earliest obstacle wins
        bEv = bT.copy()
        xEv, yEv, zEv = X.copy(), Y.copy(), Z.copy()
        con = cmin < 0.0
        if con.any():
            ci = np.nonzero(con)[0]
            Cc = C[ci]
            Cp = A['Cprev'][ui[ci]]
            with np.errstate(divide='ignore', invalid='ignore'):
                lam = np.where(Cc < 0.0,
                               np.where(np.isfinite(Cp) & (Cp > 0.0), Cp / (Cp - Cc), 0.0),
                               np.inf)
            j = np.argmin(lam + 1e-9 * np.minimum(Cc, 0.0), axis=1)
            lj = lam[np.arange(ci.size), j]
            if first:
                lj = np.ones(ci.size)
            # target body coordinates of the centre at the (linearly interpolated) contact instant
            wp, fp, bp = A['wprev'][ui[ci]], A['fprev'][ui[ci]], A['bprev'][ui[ci]]
            okp = np.isfinite(wp)
            wc = np.where(okp, wp + lj * (wT[ci] - wp), wT[ci])
            fc = np.where(okp, fp + lj * (fT[ci] - fp), fT[ci])
            bc = np.where(okp, bp + lj * (bT[ci] - bp), bT[ci])
            bEv[ci] = bc
            Tc = A['T'][ui[ci]]
            xc, yc, zc = geom.to_world(wc, fc + geom.aOut, bc, Tc, A['sig'][ui[ci], Tc])
            xEv[ci], yEv[ci], zEv[ci] = xc, yc, zc
            kinds = geom.kind[j]
            shell_hit = kinds == KIND_SHELL
            ev[ci] = EV_CONTACT
            obst[ci] = j
            # SPEC 5 "lip: target shell contact near the base edge, b < b0 + 1.5", read with b =
            # the ball-centre coordinate (SPEC 5 writes (w, b) for the centre throughout).
            cause[ci] = np.where(shell_hit,
                                 np.where(bc < geom.b0 + LIP_BAND, C_LIP, C_ROOF),
                                 np.where(kinds == KIND_CELL, C_CELL, C_FRAME))
            if shell_hit.any():
                k = np.nonzero(shell_hit)[0]
                R['contactB'][gi[ci[k]]] = geom.shell_nearest_b(wc[k], fc[k] + geom.aOut, bc[k])
            if not first:
                tev[ci] = tprev + lj * (t - tprev)

        # 2. floor
        rem = ev < 0
        m = rem & (Z < rr)
        ev[m] = EV_FLOOR
        cause[m] = C_FLOOR
        # 3. walls (slab test exactly as SPEC 5, with field.json half / wall height)
        rem = ev < 0
        m = rem & ((np.abs(X) > geom.half - rr) | (np.abs(Y) > geom.half - rr)) & (Z < geom.wallH + rr)
        ev[m] = EV_WALL
        cause[m] = C_WALL
        # 4. left the field
        shortlong = np.where(rho_in < A['smouth'][ui], C_SHORT, C_LONG).astype(np.int8)
        rem = ev < 0
        m = rem & ((np.abs(X) > geom.outLim) | (np.abs(Y) > geom.outLim))
        ev[m] = EV_OUT
        cause[m] = shortlong[m]

        # 5. mouth crossing bookkeeping and score
        rem = ev < 0
        entered = A['entered'][ui]
        if not first:
            fp = A['fprev'][ui]
            cin = rem & (fp > 0.0) & (fT <= 0.0)
            if cin.any():
                k = np.nonzero(cin)[0]
                lamc = fp[k] / (fp[k] - fT[k])
                wp = A['wprev'][ui[k]]
                bp = A['bprev'][ui[k]]
                wc = wp + lamc * (wT[k] - wp)
                bc = bp + lamc * (bT[k] - bp)
                _, insc = geom.pent(wc, bc)
                entered[k] = insc
                ok = k[insc]
                if ok.size:
                    lo = lamc[insc]
                    g = gi[ok]
                    R['tCross'][g] = tprev + lo * (t - tprev)
                    sp = A['sprev'][ui[ok]]
                    R['entrySpeed'][g] = sp + lo * (SP[ok] - sp)
                    R['entryW'][g] = wc[insc]
                    R['entryB'][g] = bc[insc]
        entered &= ~(fT > 0.0)
        m = rem & entered & (fT < -rr)
        ev[m] = EV_SCORE
        cause[m] = C_NONE

        # 6. can't score any more
        rem = ev < 0
        in_target = insT & (fT <= 0.0) & (fT >= -geom.depth)
        m = rem & (VZ < 0.0) & (Z < geom.lipZ - rr) & ~in_target
        ev[m] = EV_CANT
        cause[m] = shortlong[m]

        # 7. optional: emulate the SPEC 4 trajectory-table truncation (t > 2.6 s or rho > 5.6 m)
        if table_limits:
            rem = ev < 0
            m = rem & ((t > TABLE_T_MAX) | (rho_in > rho_trunc))
            ev[m] = EV_TRUNC
            cause[m] = shortlong[m]

        if t > tmax:
            rem = ev < 0
            ev[rem] = EV_TIMEOUT
            cause[rem] = C_TIMEOUT

        A['entered'][ui] = entered
        A['fprev'][ui] = fT
        A['wprev'][ui] = wT
        A['bprev'][ui] = bT
        A['sprev'][ui] = SP
        A['Cprev'][ui] = C

        d = np.nonzero(ev >= 0)[0]
        if d.size:
            g = gi[d]
            R['event'][g] = ev[d]
            R['cause'][g] = cause[d]
            R['obst'][g] = obst[d]
            R['hit'][g] = ev[d] == EV_SCORE
            R['tEvent'][g] = tev[d]
            R['evX'][g] = xEv[d]
            R['evY'][g] = yEv[d]
            R['evZ'][g] = zEv[d]
            R['evB'][g] = bEv[d]
            A['decided'][ui[d]] = True

    check(0.0, 0.0, A['rho'], A['z'], A['vr'], A['vz'], True)
    step = 0
    while True:
        alive = ~(A['decided'] & A['apexDone'])
        na = int(alive.sum())
        if na == 0:
            break
        if na < alive.size:
            for key in list(A.keys()):
                A[key] = A[key][alive]
        t0 = step * dt
        t1 = (step + 1) * dt
        if t0 > tmax + 2.0:           # only apex-tracking shots left and something is wrong
            break
        rho0, z0, vr0, vz0 = A['rho'], A['z'], A['vr'], A['vz']
        rho1, z1, vr1, vz1 = rk4_step(rho0, z0, vr0, vz0, A['kd'], A['klc'], A['wr'], dt)
        und = ~A['decided']
        n_sub = 1
        if und.any():
            vmax = np.maximum(np.hypot(vr0, vz0), np.hypot(vr1, vz1))[und]
            travel = vmax * dt / M_PER_IN / (step_frac * A['r'][und])
            n_sub = max(1, int(math.ceil(float(travel.max()) * 1.001)))
        for jsub in range(1, n_sub + 1):
            if jsub == n_sub:
                rs, zs, vrs, vzs = rho1, z1, vr1, vz1
            else:
                tau = jsub / n_sub
                h00 = 2 * tau ** 3 - 3 * tau ** 2 + 1
                h10 = tau ** 3 - 2 * tau ** 2 + tau
                h01 = -2 * tau ** 3 + 3 * tau ** 2
                h11 = tau ** 3 - tau ** 2
                rs = h00 * rho0 + h10 * dt * vr0 + h01 * rho1 + h11 * dt * vr1
                zs = h00 * z0 + h10 * dt * vz0 + h01 * z1 + h11 * dt * vz1
                vrs = vr0 + tau * (vr1 - vr0)
                vzs = vz0 + tau * (vz1 - vz0)
            check(t0 + jsub / n_sub * dt, t0 + (jsub - 1) / n_sub * dt, rs, zs, vrs, vzs, False)
        A['rho'], A['z'], A['vr'], A['vz'] = rho1, z1, vr1, vz1
        step += 1
    return R


# ----------------------------------------------------------------------------------
# Case handling
# ----------------------------------------------------------------------------------
GEOM_OPTION_KEYS = ('logoPanels', 'cornerBlocks', 'footBars', 'half', 'wallHeight', 'outMargin')
SIM_OPTION_KEYS = ('tableLimits',)
CASE_OPTION_KEYS = GEOM_OPTION_KEYS + SIM_OPTION_KEYS


def build_shot_arrays(geom, cases, idxs, yaw_mode='absolute'):
    rows = []
    for ci in idxs:
        case = cases[ci]
        cid = case.get('id')
        ball = str(case.get('ballId', 'pollen')).lower()
        if ball not in BALLS:
            raise ValueError('case %r: unknown ballId %r' % (cid, ball))
        r_in, kd, klc = ball_coeffs(ball)
        target = str(case.get('target', 'red')).lower()
        if target not in ('red', 'blue'):
            raise ValueError('case %r: unknown target %r' % (cid, target))
        T = 0 if target == 'red' else 1
        hs = dict(geom.up_start)
        hs.update({k: v for k, v in (case.get('hiveState') or {}).items() if k in ('red', 'blue')})
        for k in ('red', 'blue'):
            if int(hs[k]) not in (-1, 1):
                raise ValueError('case %r: hiveState.%s must be -1 or 1' % (cid, k))
        robot = case.get('robot') or {}
        x0 = float(robot.get('x', 0.0))
        y0 = float(robot.get('y', 0.0))
        h0 = float(case.get('h0', 16.0))
        S0 = float(case.get('S0', 1.0))
        mode = str(case.get('yawMode', yaw_mode)).lower()
        if mode not in ('absolute', 'offset'):
            raise ValueError('case %r: yawMode must be "absolute" or "offset"' % (cid,))
        psi_base = 0.0
        if mode == 'offset':
            psi_base = float(geom.heading_to_mouth(x0, y0, T, int(hs[target]))[0])
        for si, shot in enumerate(case.get('shots') or []):
            v0 = float(shot['v'])
            rows.append((ci, si, r_in, kd, klc, S0 * v0, v0, math.radians(float(shot['thetaDeg'])),
                         psi_base + math.radians(float(shot['yawDeg'])), x0, y0, h0,
                         T, int(hs['red']), int(hs['blue'])))
    if not rows:
        return None, []
    arr = np.array([r[2:] for r in rows], float)
    S = dict(r=arr[:, 0], kd=arr[:, 1], klc=arr[:, 2], wr=arr[:, 3], v0=arr[:, 4], theta=arr[:, 5],
             psi=arr[:, 6], x0=arr[:, 7], y0=arr[:, 8], h0=arr[:, 9], T=arr[:, 10].astype(int),
             sig=arr[:, 11:13])
    return S, [(r[0], r[1]) for r in rows]


def _num(x, nd):
    if x is None:
        return None
    x = float(x)
    if not math.isfinite(x):
        return None
    return round(x, nd)


def run_cases(cases, field, cli_opts, dt=DT, step_frac=STEP_FRAC, tmax=TMAX, extra=False,
              chunk=20000, quiet=False, yaw_mode='absolute'):
    groups = {}
    for ci, case in enumerate(cases):
        opts = dict(cli_opts)
        for k, v in (case.get('options') or {}).items():
            if k in CASE_OPTION_KEYS:
                opts[k] = v
        groups.setdefault(json.dumps(opts, sort_keys=True), []).append(ci)

    out = [{'id': c.get('id'), 'results': [None] * len(c.get('shots') or [])} for c in cases]
    total = sum(len(c.get('shots') or []) for c in cases)
    done = 0
    t_start = time.time()
    for key, idxs in groups.items():
        opts = json.loads(key)
        geom = Geometry(field, **{k: v for k, v in opts.items() if k in GEOM_OPTION_KEYS})
        table_limits = bool(opts.get('tableLimits', False))
        # chunk by whole cases so memory stays bounded
        batch = []
        count = 0
        batches = []
        for ci in idxs:
            batch.append(ci)
            count += len(cases[ci].get('shots') or [])
            if count >= chunk:
                batches.append(batch)
                batch, count = [], 0
        if batch:
            batches.append(batch)
        for b in batches:
            S, where = build_shot_arrays(geom, cases, b, yaw_mode=yaw_mode)
            if S is None:
                continue
            R = simulate(geom, S, dt=dt, step_frac=step_frac, tmax=tmax, table_limits=table_limits)
            for n, (ci, si) in enumerate(where):
                hit = bool(R['hit'][n])
                res = {'hit': hit,
                       'cause': None if hit else CAUSES[int(R['cause'][n])],
                       'apexIn': _num(R['apexFree'][n], 4),
                       'tToCell': _num(R['tCross'][n], 5) if hit else None}
                if extra:
                    ev = int(R['event'][n])
                    ob = int(R['obst'][n])
                    cause_alt = res['cause']
                    if math.isfinite(R['contactB'][n]):
                        cause_alt = 'lip' if R['contactB'][n] < geom.b0 + LIP_BAND else 'roof'
                    res.update({
                        'event': EVENTS[ev] if ev >= 0 else None,
                        'obstacle': geom.names[ob] if ob >= 0 else None,
                        'tEvent': _num(R['tEvent'][n], 5),
                        'eventXYZ': [_num(R['evX'][n], 3), _num(R['evY'][n], 3), _num(R['evZ'][n], 3)],
                        'eventB': _num(R['evB'][n], 3),
                        'contactB': _num(R['contactB'][n], 3),
                        'causeContactPoint': cause_alt,
                        'apexToEventIn': _num(R['apexEvent'][n], 4),
                        'minClearanceIn': _num(R['minClear'][n], 4),
                        'entrySpeedMps': _num(R['entrySpeed'][n], 4) if hit else None,
                        'entryWB': [_num(R['entryW'][n], 3), _num(R['entryB'][n], 3)] if hit else None,
                        'sMouthIn': _num(R['sMouth'][n], 3),
                        'psiDeg': _num(math.degrees(S['psi'][n]), 4),
                        'psi0Deg': _num(math.degrees(R['psi0'][n]), 4),
                    })
                out[ci]['results'][si] = res
            done += len(where)
            if not quiet:
                print('  %d / %d shots  (%.1f s)' % (done, total, time.time() - t_start), file=sys.stderr)
    return out


# ----------------------------------------------------------------------------------
# Self-test
# ----------------------------------------------------------------------------------
class _Report:
    def __init__(self):
        self.fail = 0
        self.n = 0

    def ok(self, name, cond, detail=''):
        self.n += 1
        if not cond:
            self.fail += 1
        print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name, ('  ' + detail) if detail else ''))

    @staticmethod
    def info(name, detail):
        print('  [INFO] %s  %s' % (name, detail))


def _brute_min_dist(P, pts, block=128):
    out = np.empty(P.shape[0])
    for i in range(0, P.shape[0], block):
        q = P[i:i + block]
        d2 = np.full(q.shape[0], np.inf)
        for j in range(0, pts.shape[0], 200000):
            s = pts[j:j + 200000]
            dd = (q[:, None, 0] - s[None, :, 0]) ** 2 + (q[:, None, 1] - s[None, :, 1]) ** 2 \
                + (q[:, None, 2] - s[None, :, 2]) ** 2
            d2 = np.minimum(d2, dd.min(axis=1))
        out[i:i + block] = np.sqrt(d2)
    return out


def _prism_surface_body(geom, amin, amax, h, cap_lo=True, cap_hi=True):
    """Dense (w, a, b) samples on a pentagonal prism surface (sides + chosen caps)."""
    pts = []
    na = int(math.ceil((amax - amin) / h)) + 1
    avals = np.linspace(amin, amax, na)
    for i in range(5):
        p = geom.V[i]
        e = geom.E[i]
        L = math.hypot(e[0], e[1])
        ne = int(math.ceil(L / h)) + 1
        tt = np.linspace(0.0, 1.0, ne)
        ew = p[0] + tt * e[0]
        eb = p[1] + tt * e[1]
        W, AA = np.meshgrid(ew, avals)
        B, _ = np.meshgrid(eb, avals)
        pts.append(np.stack([W.ravel(), AA.ravel(), B.ravel()], axis=1))
    wmin, wmax = geom.V[:, 0].min(), geom.V[:, 0].max()
    bmin, bmax = geom.V[:, 1].min(), geom.V[:, 1].max()
    gw, gb = np.meshgrid(np.arange(wmin, wmax + 1e-9, h), np.arange(bmin, bmax + 1e-9, h))
    gw, gb = gw.ravel(), gb.ravel()
    _, ins = geom.pent(gw, gb)
    gw, gb = gw[ins], gb[ins]
    for flag, aval in ((cap_lo, amin), (cap_hi, amax)):
        if flag:
            pts.append(np.stack([gw, np.full(gw.shape, aval), gb], axis=1))
    return np.concatenate(pts, axis=0)


def _body_to_world_pts(geom, P, hive, sigma):
    x, y, z = geom.to_world(P[:, 0], P[:, 1], P[:, 2], hive, sigma)
    return np.stack([x, y, z], axis=1)


def selftest():
    rep = _Report()
    field = load_field()
    geom = Geometry(field)
    hv = field['hive']
    chk = hv.get('checks', {})
    rng = np.random.default_rng(20260912)
    print('BIOBUZZ oracle self-test  (field.json: %s)' % FIELD_JSON)

    # ---- 1. lip / top / centroid mapping (SPEC 3.1, tolerance 0.02 in) ----------------
    print('Geometry mapping')
    for sigma in (1, -1):
        for hive in (0, 1):
            x, y, z = geom.to_world(0.0, geom.aOut, geom.b0, hive, sigma)
            rep.ok('lip   hive=%d sigma=%+d -> (u, z) = (%.3f, %.3f)' % (hive, sigma, sigma * y, z),
                   abs(sigma * y - 19.27) <= 0.02 and abs(z - 53.50) <= 0.02 and abs(x - geom.hx[hive]) < 1e-12)
    x, y, z = geom.to_world(0.0, geom.aOut, geom.b0 + float(hv['cell']['height']), 0, 1)
    rep.ok('top   (u, z) = (%.3f, %.3f) vs (12.27, 65.62)' % (y, z), abs(y - 12.27) <= 0.02 and abs(z - 65.62) <= 0.02)
    x, y, z = geom.mouth_centroid_world(0, 1)
    rep.ok('mouth centroid (w, b-b0) = (%.4f, %.4f) vs (0, 5.5597..5.5600)' % (geom.cw, geom.cb - geom.b0),
           abs(geom.cw) < 1e-9 and abs(geom.cb - geom.b0 - 5.5598) <= 0.001)
    rep.ok('mouth centroid (u, z) = (%.3f, %.3f) vs SPEC (16.49, 58.31)' % (y, z),
           abs(y - 16.49) <= 0.02 and abs(z - 58.31) <= 0.02)
    if 'lipUZ' in chk:
        for nm, (a_, b_) in (('lipUZ', (geom.aOut, geom.b0)),
                             ('topUZ', (geom.aOut, geom.b0 + float(hv['cell']['height']))),
                             ('mouthCentroidUZ', (geom.aOut, geom.cb))):
            _, yy, zz = geom.to_world(0.0, a_, b_, 0, 1)
            rep.ok('field.json checks.%s %s vs computed (%.3f, %.3f)' % (nm, chk[nm], yy, zz),
                   abs(yy - chk[nm][0]) <= 0.02 and abs(zz - chk[nm][1]) <= 0.02)
    _, _, zl = geom.to_world(0.0, geom.aOut, geom.b0, 0, 1)
    rep.ok('hive.lipZ %.3f vs computed %.4f' % (geom.lipZ, zl), abs(geom.lipZ - zl) <= 0.02)
    # mouth outward normal = sigma*y_hat*cos30 + z_hat*sin30 (direction of +a)
    for sigma in (1, -1):
        p0 = np.array(geom.to_world(0.0, geom.aOut, geom.cb, 0, sigma))
        p1 = np.array(geom.to_world(0.0, geom.aOut + 1.0, geom.cb, 0, sigma))
        nrm = np.array([0.0, sigma * math.cos(math.radians(30)), math.sin(math.radians(30))])
        rep.ok('mouth outward normal sigma=%+d' % sigma, np.allclose(p1 - p0, nrm, atol=1e-12))
    # body round trip
    P = rng.uniform([-80, -80, -5], [80, 80, 90], size=(2000, 3))
    err = 0.0
    for hive in (0, 1):
        for sigma in (1, -1):
            w, a, b = geom.to_body(P[:, 0], P[:, 1], P[:, 2], hive, sigma)
            Q = np.stack(geom.to_world(w, a, b, hive, sigma), axis=1)
            err = max(err, float(np.abs(Q - P).max()))
            d0 = np.linalg.norm(P[1:] - P[:-1], axis=1)
            d1 = np.sqrt(np.diff(w) ** 2 + np.diff(a) ** 2 + np.diff(b) ** 2)
            err = max(err, float(np.abs(d0 - d1).max()))
    rep.ok('world <-> body round trip and isometry (max err %.2e)' % err, err < 1e-9)

    # ---- 1b. SPEC 10: field limits and obstacle set come from field.json ------------------
    print('SPEC 10 field limits and obstacle set (defaults)')
    fld = field['field']
    fr = hv['frame']
    rep.ok('half %.2f, wall height %.2f, left-field limit %.2f = half + 6 (field.json)' %
           (geom.half, geom.wallH, geom.outLim),
           geom.half == float(fld['half']) and geom.wallH == float(fld['wallHeight'])
           and abs(geom.outLim - (float(fld['half']) + 6.0)) < 1e-12)
    exp_names = (['targetUpCell', 'targetDownCell', 'otherUpCell', 'otherDownCell', 'armBarRed', 'armBarBlue']
                 + ['leg%d' % i for i in range(len(fr['legs']))] + ['crossbar']
                 + ['cornerBlock%d' % i for i in range(len(fr['cornerBlocks']['segments']))])
    rep.ok('default obstacles: CELLs, arm bars, legs, crossbar, cornerBlocks; no footBars/logoPanels (%d)'
           % geom.K, geom.names == exp_names)
    exp_caps = ([(p_, q_, fr['tubeRadius']) for p_, q_ in fr['legs']]
                + [(fr['crossbar'][0], fr['crossbar'][1], fr['tubeRadius'])]
                + [(p_, q_, fr['cornerBlocks']['radius']) for p_, q_ in fr['cornerBlocks']['segments']])
    rep.ok('capsule endpoints and radii equal field.json (legs/crossbar r %.2f, cornerBlocks r %.2f)' %
           (fr['tubeRadius'], fr['cornerBlocks']['radius']),
           np.array_equal(geom.cap_A, np.array([c[0] for c in exp_caps], float))
           and np.array_equal(geom.cap_B, np.array([c[1] for c in exp_caps], float))
           and np.array_equal(geom.cap_r, np.array([c[2] for c in exp_caps], float)))
    rep.ok('arm bar = field.json %d-point polyline, radius %.2f' % (len(hv['armBar']['ab']), geom.arm_r),
           np.array_equal(geom.arm_P, np.array(hv['armBar']['ab'], float))
           and geom.arm_r == float(hv['armBar']['radius']))
    gopt = Geometry(field, footBars=True, logoPanels=True)
    rep.ok('opt-in footBars + logoPanels append exactly those obstacles',
           gopt.names == exp_names + ['footBar0', 'footBar1', 'logoPanel0', 'logoPanel1'])
    gnc = Geometry(field, cornerBlocks=False)
    rep.ok('cornerBlocks=False removes only the corner blocks',
           gnc.names == [nm for nm in exp_names if not nm.startswith('cornerBlock')])

    # ---- 2. pentagon distance edge cases -------------------------------------------------
    print('Pentagon distance')
    b0 = geom.b0
    cases = [((0.0, b0 + 5.0), True, 5.0),           # inside, nearest edge = base
             ((0.0, b0 - 1.0), False, 1.0),          # below base
             ((15.0, b0 + 3.0), False, 5.0),         # right of side
             ((0.0, b0 + 16.0), False, 2.0),         # above apex vertex
             ((10.0, b0 + 3.0), True, 0.0),          # on right side edge
             ((-12.0, b0 - 3.0), False, math.hypot(2.0, 3.0)),  # beyond bottom-left corner
             ((0.0, b0 + 14.0), True, 0.0),          # apex vertex itself
             ((-10.0, b0), True, 0.0)]               # base corner
    for (w, b), exp_in, exp_d in cases:
        de, ins = geom.pent(np.array([w]), np.array([b]))
        rep.ok('pent(%.2f, b0%+.2f): inside=%s dEdge=%.4f' % (w, b - b0, bool(ins[0]), de[0]),
               bool(ins[0]) == exp_in and abs(de[0] - exp_d) < 1e-9)
    # roof edge: point along outward normal of the right roof edge
    e = geom.V[3] - geom.V[2]
    mid = geom.V[2] + 0.5 * e
    nout = np.array([e[1], -e[0]]) / np.hypot(*e)
    p = mid + 1.25 * nout
    de, ins = geom.pent(np.array([p[0]]), np.array([p[1]]))
    rep.ok('pent roof-edge normal offset 1.25: dEdge=%.6f inside=%s' % (de[0], bool(ins[0])),
           (not ins[0]) and abs(de[0] - 1.25) < 1e-9)
    # brute-force pentagon boundary distance
    bw = np.concatenate([np.linspace(geom.V[i, 0], geom.V[(i + 1) % 5, 0], 4001) for i in range(5)])
    bb = np.concatenate([np.linspace(geom.V[i, 1], geom.V[(i + 1) % 5, 1], 4001) for i in range(5)])
    Q = rng.uniform([-16, b0 - 6], [16, b0 + 20], size=(400, 2))
    de, ins = geom.pent(Q[:, 0], Q[:, 1])
    brute = np.sqrt(((Q[:, None, 0] - bw) ** 2 + (Q[:, None, 1] - bb) ** 2).min(axis=1))
    rep.ok('pent dEdge vs brute force (max |diff| %.2e)' % np.abs(de - brute).max(),
           np.abs(de - brute).max() < 5e-3 and np.all(de <= brute + 1e-9))

    # ---- 3. prism / shell / capsule / arm bar distances vs brute force --------------------
    print('3-D distances vs brute-force sampling (grid 0.1 in, tol 0.08 in)')
    h = 0.1
    tol = 0.08
    sigma_r, sigma_b = -1, 1
    sig = np.array([[sigma_r, sigma_b]])
    zero = lambda n: np.zeros(n)

    def query_box(pts_world, n, pad=4.0):
        lo = pts_world.min(axis=0) - pad
        hi = pts_world.max(axis=0) + pad
        return rng.uniform(lo, hi, size=(n, 3))

    def compare(name, col, surf_body, hive, sigma, target, solid_ranges, nq=500, extra_q=None):
        surf = _body_to_world_pts(geom, surf_body, hive, sigma)
        Qw = query_box(surf, nq)
        if extra_q is not None:
            Qw = np.concatenate([Qw, extra_q], axis=0)
        n = Qw.shape[0]
        C, *_ = geom.clearances(Qw[:, 0], Qw[:, 1], Qw[:, 2], np.repeat(sig, n, axis=0),
                                np.full(n, target), zero(n))
        d = C[:, col]
        brute = _brute_min_dist(Qw, surf)
        if solid_ranges is not None:
            w, a, b = geom.to_body(Qw[:, 0], Qw[:, 1], Qw[:, 2], hive, sigma)
            _, ins = geom.pent(w, b)
            inside = ins & (a >= solid_ranges[0]) & (a <= solid_ranges[1])
            brute = np.where(inside, 0.0, brute)
            n_in = int(inside.sum())
        else:
            n_in = 0
        diff = brute - d
        rep.ok('%s: %d pts (%d inside), brute-analytic in [%.2e, %.4f] (need >= -1e-9, <= %.2f)' %
               (name, n, n_in, diff.min(), diff.max(), tol),
               np.all(diff >= -1e-9) and diff.max() <= tol)
        return Qw

    # target = red (T = 0), red sigma = -1
    shell_surf = _prism_surface_body(geom, geom.aIn, geom.aOut, h, cap_lo=True, cap_hi=False)

    def cavity_points(hive, sigma, n=300):
        """points in and around the CELL cavity: near the mouth, the walls and the back wall"""
        w = rng.uniform(-11, 11, n)
        b = rng.uniform(geom.b0 - 1, geom.b0 + 15, n)
        a = rng.uniform(geom.aIn - 1.5, geom.aOut + 3, n)
        return np.stack(geom.to_world(w, a, b, hive, sigma), axis=1)

    compare('target up-CELL shell (red, sigma=-1)', 0, shell_surf, 0, sigma_r, 0, None,
            extra_q=cavity_points(0, sigma_r))
    down_surf = _prism_surface_body(geom, -geom.aOut, -geom.aIn, h)
    compare('target down-CELL solid prism (red)', 1, down_surf, 0, sigma_r, 0, (-geom.aOut, -geom.aIn))
    up_surf = _prism_surface_body(geom, geom.aIn, geom.aOut, h)
    compare('other up-CELL solid prism (blue, sigma=+1)', 2, up_surf, 1, sigma_b, 0, (geom.aIn, geom.aOut))
    compare('other down-CELL solid prism (blue)', 3, down_surf, 1, sigma_b, 0, (-geom.aOut, -geom.aIn))
    # swapped target: blue shell is column 0 when T = 1
    Qs = compare('target up-CELL shell (blue, sigma=+1)', 0, shell_surf, 1, sigma_b, 1, None,
                 extra_q=cavity_points(1, sigma_b))

    # shell nearest point: lies on the sampled surface and realises the analytic distance
    ws, as_, bs = geom.to_body(Qs[:, 0], Qs[:, 1], Qs[:, 2], 1, sigma_b)
    de, ins = geom.pent(ws, bs)
    dsh = geom.shell(de, ins, as_)
    nw, na, nb = geom.shell_nearest(ws, as_, bs)
    real = np.sqrt((ws - nw) ** 2 + (as_ - na) ** 2 + (bs - nb) ** 2)
    npw = _body_to_world_pts(geom, np.stack([nw, na, nb], axis=1), 1, sigma_b)
    onsurf = _brute_min_dist(npw, _body_to_world_pts(geom, shell_surf, 1, sigma_b))
    rep.ok('shell nearest point: |P - nearest| = analytic distance (max err %.1e), on surface (max %.3f in)'
           % (np.abs(real - dsh).max(), onsurf.max()),
           np.abs(real - dsh).max() < 1e-9 and onsurf.max() <= tol)

    # SPEC 5 as written has no back wall inside the cavity; show where that matters
    gw, gb = np.meshgrid(np.linspace(-10, 10, 801), np.linspace(geom.b0, geom.b0 + 14, 561))
    de_g, in_g = geom.pent(gw.ravel(), gb.ravel())
    inrad = float(de_g[in_g].max()) + 0.02        # grid estimate of the inradius, rounded up
    n = 20000
    w_ = rng.uniform(-10, 10, n)
    b_ = rng.uniform(geom.b0, geom.b0 + 14, n)
    f_ = rng.uniform(-(geom.depth - inrad), 3.0, n)
    de, ins = geom.pent(w_, b_)
    same = np.abs(geom.shell(de, ins, f_ + geom.aOut) - geom.shell_spec_literal(de, ins, f_ + geom.aOut)).max()
    rmax = max(ball_coeffs(k)[0] for k in BALLS)
    rep.ok('shell == SPEC-5-literal for f >= -(depth - inradius) = %.3f (max diff %.1e); scoring decided by '
           'f >= -%.2f' % (-(geom.depth - inrad), same, rmax * (1 + STEP_FRAC)),
           same == 0.0 and geom.depth - inrad > rmax * (1 + STEP_FRAC))
    f_ = rng.uniform(-geom.depth, -(geom.depth - inrad), n)
    lit = geom.shell_spec_literal(de, ins, f_ + geom.aOut)
    fix = geom.shell(de, ins, f_ + geom.aOut)
    rep.info('SPEC-5-literal shell near the back wall', 'overestimates by up to %.2f in (inradius %.3f in)'
             % ((lit - fix).max(), inrad))
    # continuity across f = 0 and f = -depth (the SPEC-literal form jumps at f = -depth)
    eps = 1e-9
    jumps = []
    for a_edge in (geom.aOut, geom.aIn):
        lo = geom.shell(de, ins, np.full(n, a_edge - eps))
        hi = geom.shell(de, ins, np.full(n, a_edge + eps))
        jumps.append(np.abs(lo - hi).max())
    rep.ok('shell distance continuous at f = 0 and f = -depth (max jumps %.1e, %.1e)' % tuple(jumps),
           max(jumps) < 1e-6)

    # arm bar: sample capsule surfaces is awkward; compare distance-to-polyline + radius
    for hive, sgm, col, nm in ((0, sigma_r, 4, 'armBarRed'), (1, sigma_b, 5, 'armBarBlue')):
        pl = []
        for i in range(len(geom.arm_A)):
            tt = np.linspace(0, 1, 3000)[:, None]
            seg = geom.arm_A[i] + tt * geom.arm_D[i]
            pl.append(np.stack([np.zeros(len(tt)), seg[:, 0], seg[:, 1]], axis=1))
        pl = _body_to_world_pts(geom, np.concatenate(pl), hive, sgm)
        Qw = query_box(pl, 500)
        n = Qw.shape[0]
        C, *_ = geom.clearances(Qw[:, 0], Qw[:, 1], Qw[:, 2], np.repeat(sig, n, axis=0), np.zeros(n, int), zero(n))
        d = C[:, col] + geom.arm_r
        brute = _brute_min_dist(Qw, pl)
        rep.ok('%s polyline distance vs brute (max diff %.2e)' % (nm, np.abs(brute - d).max()),
               np.all(brute - d >= -1e-9) and np.abs(brute - d).max() < 0.01)

    geomL = Geometry(field, logoPanels=True, footBars=True)
    base = 6
    for i in range(geomL.cap_A.shape[0]):
        tt = np.linspace(0, 1, 20001)[:, None]
        seg = geomL.cap_A[i] + tt * geomL.cap_D[i]
        Qw = query_box(seg, 300, pad=6.0)
        n = Qw.shape[0]
        C, *_ = geomL.clearances(Qw[:, 0], Qw[:, 1], Qw[:, 2], np.repeat(sig, n, axis=0), np.zeros(n, int), zero(n))
        d = C[:, base + i] + geomL.cap_r[i]
        brute = _brute_min_dist(Qw, seg)
        rep.ok('capsule %-12s vs brute (max diff %.2e)' % (geomL.names[base + i], np.abs(brute - d).max()),
               np.all(brute - d >= -1e-9) and np.abs(brute - d).max() < 0.005)
    for qi, quad in enumerate(geomL.quads):
        Qv = quad[0]
        uu, vv = np.meshgrid(np.linspace(0, 1, 600), np.linspace(0, 1, 160))
        uu, vv = uu.ravel()[:, None], vv.ravel()[:, None]
        surf = (1 - vv) * ((1 - uu) * Qv[0] + uu * Qv[1]) + vv * ((1 - uu) * Qv[3] + uu * Qv[2])
        Qw = query_box(surf, 400)
        n = Qw.shape[0]
        C, *_ = geomL.clearances(Qw[:, 0], Qw[:, 1], Qw[:, 2], np.repeat(sig, n, axis=0), np.zeros(n, int), zero(n))
        col = base + geomL.cap_A.shape[0] + qi
        d = C[:, col] + quad[5]
        brute = _brute_min_dist(Qw, surf)
        rep.ok('logo panel %d vs brute (max diff %.3f)' % (qi, (brute - d).max()),
               np.all(brute - d >= -1e-9) and (brute - d).max() < 0.06)

    # ---- 4. flight ----------------------------------------------------------------------
    print('Flight')
    v0, th = 7.0, math.radians(55.0)
    rho, z, vr, vz = np.zeros(1), np.zeros(1), np.array([v0 * math.cos(th)]), np.array([v0 * math.sin(th)])
    nsteps = int(round(1.0 / DT))
    for _ in range(nsteps):
        rho, z, vr, vz = rk4_step(rho, z, vr, vz, 0.0, 0.0, 0.0, DT)
    ex_r = v0 * math.cos(th) * 1.0
    ex_z = v0 * math.sin(th) * 1.0 - 0.5 * G
    err = math.hypot(rho[0] - ex_r, z[0] - ex_z)
    rep.ok('no-air RK4 vs analytic parabola at t = 1 s: error %.3e m (< 1 mm)' % err, err < 1e-3)

    r_in, kd, klc = ball_coeffs('pollen')
    vt = math.sqrt(G / kd)
    rho, z, vr, vz = np.zeros(1), np.zeros(1), np.array([0.0]), np.array([0.0])
    for _ in range(int(round(12.0 / DT))):
        rho, z, vr, vz = rk4_step(rho, z, vr, vz, kd, klc, 0.0, DT)
    rep.ok('POLLEN terminal speed sqrt(g/kd) = %.3f m/s, 12 s drop -> %.3f m/s (15.1 +/- 0.1)' % (vt, -vz[0]),
           abs(vt - 15.1) <= 0.1 and abs(-vz[0] - 15.1) <= 0.1)
    rn, kdn, _ = ball_coeffs('nectar')
    rep.info('NECTAR', 'r = %.4f in, kd = %.5f 1/m, terminal %.3f m/s' % (rn, kdn, math.sqrt(G / kdn)))

    # full simulate() with air off: free apex must match the analytic apex
    S = dict(r=np.array([r_in]), kd=np.array([0.0]), klc=np.array([0.0]), wr=np.array([0.0]),
             v0=np.array([6.0]), theta=np.array([math.radians(62.0)]), psi=np.array([math.radians(180.0)]),
             x0=np.array([-40.0]), y0=np.array([-50.0]), h0=np.array([10.0]), T=np.array([0]),
             sig=np.array([[-1, 1]]))
    R = simulate(geom, S)
    apex_exact = 10.0 + (6.0 * math.sin(math.radians(62.0))) ** 2 / (2 * G) / M_PER_IN
    rep.ok('simulate() no-air apex %.4f in vs analytic %.4f in' % (R['apexFree'][0], apex_exact),
           abs(R['apexFree'][0] - apex_exact) < 0.005)

    # ---- 5. end-to-end sanity -----------------------------------------------------------
    print('End-to-end (red target, sigma = -1, POLLEN, S0 = 1)')
    hx = geom.hx[0]

    def shots(robot, h0, lst, ball='pollen', S0=1.0, target='red', hs=None, g=geom):
        case = {'id': 0, 'ballId': ball, 'target': target, 'hiveState': hs or {'red': -1, 'blue': 1},
                'robot': {'x': robot[0], 'y': robot[1]}, 'h0': h0, 'S0': S0,
                'shots': [{'thetaDeg': a, 'v': v, 'yawDeg': y} for a, v, y in lst]}
        return run_cases([case], field, {}, extra=True, quiet=True)[0]['results']

    r = shots((hx, -48.0), 16.0, [(70.5, 5.05, 90.0)])[0]
    rep.ok('reference in-plane shot u0=48, theta 70.5, v 5.05 -> hit (%s, tToCell %s, apex %s, clear %s)' %
           (r['cause'], r['tToCell'], r['apexIn'], r['minClearanceIn']), r['hit'])
    r = shots((hx, -48.0), 16.0, [(70.5, 3.5, 90.0)])[0]
    rep.ok('same, v 3.5 -> short (%s/%s)' % (r['event'], r['cause']), (not r['hit']) and r['cause'] == 'short')
    r = shots((hx, -48.0), 16.0, [(45.0, 8.0, 90.0)])[0]
    rep.ok('same, theta 45 v 8 -> miss not short (%s/%s %s)' % (r['event'], r['cause'], r['obstacle']),
           (not r['hit']) and r['cause'] != 'short')
    r = shots((hx, 48.0), 16.0, [(70.5, 5.05, -90.0)], hs={'red': 1, 'blue': 1})[0]
    rep.ok('mirrored (red sigma=+1, robot y=+48, heading -90) -> hit', r['hit'])
    r = shots((-hx, 48.0), 16.0, [(70.5, 5.05, -90.0)], target='blue')[0]
    rep.ok('blue target sigma=+1 mirrored in x -> hit', r['hit'])
    r = shots((hx, 0.0), 29.0, [(89.0, 6.0, 90.0)])[0]
    rep.ok('under the pivot, straight up -> contact (%s/%s %s)' % (r['event'], r['cause'], r['obstacle']),
           (not r['hit']) and r['event'] == 'contact')
    r = shots((-62.0, 0.0), 4.0, [(10.0, 5.0, 180.0)])[0]
    rep.ok('robot near red wall, flat shot at wall -> wall (%s)' % r['cause'], r['cause'] == 'wall')
    r = shots((-12.75, -48.0), 16.0, [(70.5, 5.05, 90.0 + 25.0)])[0]
    rep.ok('yaw 25 deg off -> miss (%s/%s %s)' % (r['event'], r['cause'], r['obstacle']), not r['hit'])

    # yawMode "offset" == absolute heading psi0 + offset, for an off-axis robot
    rob = (hx + 20.0, -40.0)
    psi0 = math.degrees(float(geom.heading_to_mouth(rob[0], rob[1], 0, -1)[0]))
    lst = [(th_, v_, dy) for th_, v_, dy in ((68.0, 5.1, 0.0), (72.0, 5.0, 1.5), (66.0, 5.3, -2.0))]
    case_abs = {'id': 'a', 'ballId': 'pollen', 'target': 'red', 'robot': {'x': rob[0], 'y': rob[1]}, 'h0': 16.0,
                'S0': 1.0, 'shots': [{'thetaDeg': t_, 'v': v_, 'yawDeg': psi0 + dy} for t_, v_, dy in lst]}
    case_off = dict(case_abs, id='o', yawMode='offset',
                    shots=[{'thetaDeg': t_, 'v': v_, 'yawDeg': dy} for t_, v_, dy in lst])
    ra, ro = [c['results'] for c in run_cases([case_abs, case_off], field, {}, quiet=True)]
    rep.ok('yawMode offset == absolute psi0 + offset (psi0 = %.3f deg, hits %s)' % (psi0, [q['hit'] for q in ra]),
           len(ra) == len(ro) and all(p_['hit'] == q_['hit'] and p_['cause'] == q_['cause']
                                      and abs(p_['apexIn'] - q_['apexIn']) < 1e-3
                                      and (p_['tToCell'] is None) == (q_['tToCell'] is None)
                                      and abs((p_['tToCell'] or 0) - (q_['tToCell'] or 0)) < 1e-4
                                      for p_, q_ in zip(ra, ro)))

    # contact classification at the interpolated first-touch instant
    lst = [(a_, v_, y_) for a_ in (60.0, 66.0, 72.0, 78.0) for v_ in np.arange(4.4, 6.01, 0.04)
           for y_ in (90.0, 97.0, 104.0, 111.0)]
    res = shots((hx - 4.0, -46.0), 16.0, lst)
    cnt = {}
    agree = 0
    nshell = 0
    band_ok = True
    for q in res:
        cnt[q['cause']] = cnt.get(q['cause'], 0) + 1
        if q['cause'] in ('lip', 'roof'):
            nshell += 1
            agree += q['cause'] == q['causeContactPoint']
            band_ok &= (q['eventB'] < geom.b0 + LIP_BAND) == (q['cause'] == 'lip')
            band_ok &= q['obstacle'] == 'targetUpCell'
    rep.ok('lip/roof = target shell contact split at centre b0 + 1.5 (%d shell contacts)' % nshell,
           band_ok and nshell > 0)
    rep.info('cause mix over %d shots' % len(res), ', '.join('%s %d' % (k, v) for k, v in
                                                           sorted(cnt.items(), key=lambda kv: str(kv[0]))))
    rep.info('lip/roof reading', 'centre-b and contact-point-b readings agree on %d of %d shell contacts'
             % (agree, nshell))

    # ---- 6. SPEC 9 in-plane windows (informational: model differs from the 2-D reference) ----
    print('SPEC 9 in-plane windows (informational; expected within one table step)')

    def window(robot, fixed, var_vals, vary):
        lst = [((fixed, x, 90.0) if vary == 'v' else (x, fixed, 90.0)) for x in var_vals]
        res = shots(robot, 16.0, lst)
        hits = np.array([q['hit'] for q in res])
        if not hits.any():
            return None
        idx = np.nonzero(hits)[0]
        return float(var_vals[idx[0]]), float(var_vals[idx[-1]]), int(hits.sum()), len(idx) == idx[-1] - idx[0] + 1

    vv = np.round(np.arange(4.30, 5.70001, 0.01), 4)
    tv = np.round(np.arange(60.0, 85.0001, 0.1), 4)
    for label, robot, fixed, vals, vary, exp in (
            ('u0=48 theta=70.5 row, v', (hx, -48.0), 70.5, vv, 'v', (4.80, 5.31)),
            ('u0=48 v=5.05 column, theta', (hx, -48.0), 5.05, tv, 'theta', (65.5, 75.5)),
            ('u0=30 theta=81.5 row, v', (hx, -30.0), 81.5, vv, 'v', (4.57, 5.20))):
        wdw = window(robot, fixed, vals, vary)
        if wdw is None:
            rep.info(label, 'no hits (expected ~[%.2f, %.2f])' % exp)
            continue
        step = 0.02 * exp[0] if vary == 'v' else 1.0
        agree = abs(wdw[0] - exp[0]) <= step + 1e-9 and abs(wdw[1] - exp[1]) <= step + 1e-9
        rep.info(label, 'oracle [%.2f, %.2f] (contiguous=%s) vs SPEC ~[%.2f, %.2f] -> %s' %
                 (wdw[0], wdw[1], wdw[3], exp[0], exp[1], 'within one step' if agree else 'DIFFERS'))

    # ---- 7. discretisation convergence (informational) ------------------------------------
    print('Convergence (informational): dt 0.5 ms / 0.2 r  vs  dt 0.25 ms / 0.1 r')
    n = 600
    case = {'id': 'conv', 'ballId': 'pollen', 'target': 'red', 'hiveState': {'red': -1, 'blue': 1},
            'robot': {'x': hx + 6.0, 'y': -44.0}, 'h0': 16.0, 'S0': 1.0,
            'shots': [{'thetaDeg': float(a), 'v': float(v), 'yawDeg': float(y)} for a, v, y in
                      zip(rng.uniform(60, 82, n), rng.uniform(4.4, 5.8, n), rng.uniform(88, 104, n))]}
    t0 = time.time()
    A1 = run_cases([case], field, {}, extra=True, quiet=True)[0]['results']
    t1 = time.time()
    A2 = run_cases([case], field, {}, dt=0.25e-3, step_frac=0.1, extra=True, quiet=True)[0]['results']
    t2 = time.time()
    flips = [(p, q) for p, q in zip(A1, A2) if p['hit'] != q['hit']]
    cause_d = sum(1 for p, q in zip(A1, A2) if p['cause'] != q['cause'])
    near = sum(1 for p, q in flips if min(abs(p['minClearanceIn'] or 0), abs(q['minClearanceIn'] or 0)) < 0.02)
    apex_d = max(abs(p['apexIn'] - q['apexIn']) for p, q in zip(A1, A2))
    tt = [abs(p['tToCell'] - q['tToCell']) for p, q in zip(A1, A2) if p['hit'] and q['hit']]
    rep.info('convergence', '%d shots, %d hits; hit flips %d (%d with |clearance| < 0.02 in); cause diffs %d; '
             'max apex diff %.4f in; max tToCell diff %.5f s; %.2f s vs %.2f s' %
             (n, sum(p['hit'] for p in A1), len(flips), near, cause_d, apex_d, max(tt) if tt else 0.0,
              t1 - t0, t2 - t1))

    print('%d checks, %d failed' % (rep.n, rep.fail))
    return rep.fail == 0


# ----------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description='BIOBUZZ Shot Sim independent Python oracle (SPEC 3-5).')
    ap.add_argument('cases', nargs='?')
    ap.add_argument('out', nargs='?')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--extra', action='store_true', help='add diagnostic fields to every result')
    ap.add_argument('--logo-panels', action='store_true')
    ap.add_argument('--foot-bars', action='store_true')
    ap.add_argument('--no-corner-blocks', action='store_true')
    ap.add_argument('--half', type=float)
    ap.add_argument('--wall-height', type=float)
    ap.add_argument('--out-margin', type=float, default=6.0)
    ap.add_argument('--table-limits', action='store_true')
    ap.add_argument('--yaw-offset', action='store_true')
    ap.add_argument('--dt', type=float, default=DT)
    ap.add_argument('--step-frac', type=float, default=STEP_FRAC)
    ap.add_argument('--tmax', type=float, default=TMAX)
    ap.add_argument('--chunk', type=int, default=20000)
    ap.add_argument('--field', default=FIELD_JSON)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)

    if args.selftest:
        return 0 if selftest() else 1
    if not args.cases or not args.out:
        ap.error('need <cases.json> <out.json> (or --selftest)')

    field = load_field(args.field)
    with open(args.cases, 'r', encoding='utf-8') as fh:
        cases = json.load(fh)
    if isinstance(cases, dict):
        cases = cases.get('cases', [cases])
    opts = {'logoPanels': bool(args.logo_panels), 'cornerBlocks': not args.no_corner_blocks,
            'footBars': bool(args.foot_bars), 'outMargin': args.out_margin,
            'tableLimits': bool(args.table_limits)}
    if args.half is not None:
        opts['half'] = args.half
    if args.wall_height is not None:
        opts['wallHeight'] = args.wall_height
    t0 = time.time()
    out = run_cases(cases, field, opts, dt=args.dt, step_frac=args.step_frac, tmax=args.tmax,
                    extra=args.extra, chunk=args.chunk, quiet=args.quiet,
                    yaw_mode='offset' if args.yaw_offset else 'absolute')
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, indent=1)
    if not args.quiet:
        nshots = sum(len(c['results']) for c in out)
        nhit = sum(1 for c in out for r in c['results'] if r and r['hit'])
        print('oracle: %d cases, %d shots, %d hits, %.2f s -> %s' %
              (len(out), nshots, nhit, time.time() - t0, args.out), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
