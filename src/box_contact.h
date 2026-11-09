// Box vs capsule / sphere contact, the geometry GJK + EPA compute for it (box_contact.cpp).
//
// Bullet treats both shapes as a core plus a margin: the box as its implicit half extents h (btBoxShape keeps
// half extent - margin) rounded by its margin mb, the capsule as its axis segment rounded by r (a sphere is a segment
// of length 0). GJK finds the closest points of the cores and moves them out by the margins; when the cores overlap,
// EPA finds the shortest translation that separates the rounded shapes. Both have a closed form here:
//   - cores apart: the closest point of the segment to the box. Its squared distance is convex and piecewise
//     quadratic in the segment parameter, with breaks where a coordinate crosses a face plane, so the minimum is
//     found exactly piece by piece.
//   - cores overlapping: the separating-axis depth. Box (+) segment is a zonotope whose face normals are the 3 box
//     axes and the 3 cross products of the segment with them, so the smallest of those 6 depths (both directions) is
//     the exact penetration depth, the same value EPA converges to.
// One contact point per call, like the convex-convex algorithm with perturbation off (the game's setting).
#pragma once

#include <algorithm>
#include <cmath>

namespace box_contact {

struct Contact {
    double s[3];  // unit direction, world: moving the other shape along +s separates it from the box
    double pen;   // penetration depth along s (> 0 overlapping, < 0 apart by -pen)
    double q[3];  // the other shape's deepest surface point, world (the box's is q + s * pen)
    bool deep;    // the cores overlapped (the EPA case)
};

// tr: btTransform, double precision (basis rows at [0..2], [4..6], [8..10], origin at [12..14]).
inline void to_local(const double* tr, const double* p, double* out)
{
    double v0 = p[0] - tr[12], v1 = p[1] - tr[13], v2 = p[2] - tr[14];
    for (int j = 0; j < 3; j++) out[j] = tr[j] * v0 + tr[4 + j] * v1 + tr[8 + j] * v2;
}
inline void dir_to_world(const double* tr, const double* l, double* out)
{
    for (int i = 0; i < 3; i++) out[i] = tr[4 * i] * l[0] + tr[4 * i + 1] * l[1] + tr[4 * i + 2] * l[2];
}

// Squared distance of a + t d to the box [-h, h].
inline double seg_box_d2(const double* a, const double* d, const double* h, double t)
{
    double f = 0;
    for (int k = 0; k < 3; k++) {
        double x = a[k] + t * d[k];
        double e = x > h[k] ? x - h[k] : x < -h[k] ? x + h[k] : 0.0;
        f += e * e;
    }
    return f;
}

// Closest points of segments p + t u (t in [0, 1]) and e + v w (v in [-l, l], w unit).
inline void seg_line_closest(const double* p, const double* u, const double* e, const double* w, double l, double& t, double& v)
{
    double r[3] = {p[0] - e[0], p[1] - e[1], p[2] - e[2]};
    double uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2], uw = u[0] * w[0] + u[1] * w[1] + u[2] * w[2];
    double ur = u[0] * r[0] + u[1] * r[1] + u[2] * r[2], wr = w[0] * r[0] + w[1] * r[1] + w[2] * r[2];
    double den = uu - uw * uw; // |u|^2 |w|^2 - (u.w)^2 with |w| = 1
    t = den > 1e-12 * uu ? std::clamp((uw * wr - ur) / den, 0.0, 1.0) : 0.5;
    v = std::clamp(wr + t * uw, -l, l);
    t = uu > 0 ? std::clamp((v * uw - ur) / uu, 0.0, 1.0) : 0.0;
}

// Box (transform tr, core half extents h, margin mb) vs the segment p0..p1 (world) with radius r.
// False if the input is not usable (not finite, negative extents): the caller keeps Bullet's own algorithm.
inline bool box_segment(const double* tr, const double* h, double mb, const double* p0, const double* p1, double r, Contact& c)
{
    double a[3], b[3], d[3];
    to_local(tr, p0, a);
    to_local(tr, p1, b);
    for (int k = 0; k < 3; k++) d[k] = b[k] - a[k];
    double chk = a[0] + a[1] + a[2] + d[0] + d[1] + d[2] + h[0] + h[1] + h[2] + mb + r;
    if (!std::isfinite(chk) || h[0] < 0 || h[1] < 0 || h[2] < 0 || mb < 0 || r < 0) return false;

    // Cores apart? Minimise the convex piecewise quadratic over its pieces.
    double ts[8];
    int n = 0;
    ts[n++] = 0;
    ts[n++] = 1;
    for (int k = 0; k < 3; k++) {
        if (d[k] == 0) continue;
        for (int sg = -1; sg <= 1; sg += 2) {
            double t = (sg * h[k] - a[k]) / d[k];
            if (t > 0 && t < 1) ts[n++] = t;
        }
    }
    std::sort(ts, ts + n);
    double bt = 0, bf = seg_box_d2(a, d, h, 0);
    for (int i = 0; i + 1 < n; i++) {
        double t0 = ts[i], t1 = ts[i + 1], tm = 0.5 * (t0 + t1);
        double A = 0, B = 0; // f(t) = A t^2 + B t + C on this piece
        for (int k = 0; k < 3; k++) {
            double x = a[k] + tm * d[k];
            if (x > h[k] || x < -h[k]) {
                double e = a[k] - (x > 0 ? h[k] : -h[k]);
                A += d[k] * d[k];
                B += 2 * e * d[k];
            }
        }
        double t = A > 0 ? std::clamp(-B / (2 * A), t0, t1) : t0;
        for (double tt : {t, t1}) {
            double f = seg_box_d2(a, d, h, tt);
            if (f < bf) bf = f, bt = tt;
        }
    }

    double ls[3], lq[3];
    if (bf > 1e-18) {
        double cs[3], cb[3];
        for (int k = 0; k < 3; k++) cs[k] = a[k] + bt * d[k], cb[k] = std::clamp(cs[k], -h[k], h[k]);
        double dist = std::sqrt(bf);
        for (int k = 0; k < 3; k++) ls[k] = (cs[k] - cb[k]) / dist, lq[k] = cs[k] - ls[k] * r;
        c.pen = mb + r - dist;
        c.deep = false;
    } else {
        // Cores overlap: separating axes of box (+) segment.
        double best = INFINITY, axis[3] = {0, 0, 0};
        int kind = -1, kk = 0; // kind 0 face axis kk, 1 cross axis segment x e_kk
        for (int k = 0; k < 3; k++) {
            for (int sg = -1; sg <= 1; sg += 2) {
                double lo = std::min(sg * a[k], sg * b[k]);
                double pen = h[k] + mb + r - lo;
                if (pen < best) {
                    best = pen, kind = 0, kk = k;
                    axis[0] = axis[1] = axis[2] = 0;
                    axis[k] = sg;
                }
            }
        }
        double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        for (int k = 0; k < 3 && dl > 0; k++) {
            double e[3] = {0, 0, 0};
            e[k] = 1;
            double nx[3] = {d[1] * e[2] - d[2] * e[1], d[2] * e[0] - d[0] * e[2], d[0] * e[1] - d[1] * e[0]};
            double nl = std::sqrt(nx[0] * nx[0] + nx[1] * nx[1] + nx[2] * nx[2]);
            if (nl < 1e-9 * dl) continue; // segment along this box axis: same as the face axes
            for (double& x : nx) x /= nl;
            double cpos = a[0] * nx[0] + a[1] * nx[1] + a[2] * nx[2]; // the whole segment projects here
            double hb = std::fabs(nx[0]) * h[0] + std::fabs(nx[1]) * h[1] + std::fabs(nx[2]) * h[2];
            for (int sg = -1; sg <= 1; sg += 2) {
                double pen = hb + mb + r - sg * cpos;
                if (pen < best) {
                    best = pen, kind = 1, kk = k;
                    for (int j = 0; j < 3; j++) axis[j] = sg * nx[j];
                }
            }
        }
        double P[3];
        if (kind == 0) {
            // the segment's deepest point along -axis among the part within the box's other two slabs
            double tlo = 0, thi = 1;
            for (int j = 0; j < 3; j++) {
                if (j == kk) continue;
                if (d[j] == 0) continue; // inside that slab for every t (the cores overlap)
                double u0 = (-h[j] - a[j]) / d[j], u1 = (h[j] - a[j]) / d[j];
                if (u0 > u1) std::swap(u0, u1);
                tlo = std::max(tlo, u0), thi = std::min(thi, u1);
            }
            if (tlo > thi) tlo = thi = bt;
            double sd = axis[kk] * d[kk];
            double t = sd > 0 ? tlo : sd < 0 ? thi : 0.5 * (tlo + thi);
            for (int j = 0; j < 3; j++) P[j] = a[j] + t * d[j];
        } else {
            // the box edge along e_kk on the axis side, against the segment
            double E[3], w[3] = {0, 0, 0};
            for (int j = 0; j < 3; j++) E[j] = j == kk ? 0 : (axis[j] >= 0 ? h[j] : -h[j]);
            w[kk] = 1;
            double t, v;
            seg_line_closest(a, d, E, w, h[kk], t, v);
            for (int j = 0; j < 3; j++) P[j] = a[j] + t * d[j];
        }
        for (int k = 0; k < 3; k++) ls[k] = axis[k], lq[k] = P[k] - axis[k] * r;
        c.pen = best;
        c.deep = true;
    }
    dir_to_world(tr, ls, c.s);
    dir_to_world(tr, lq, c.q);
    for (int k = 0; k < 3; k++) c.q[k] += tr[12 + k];
    return std::isfinite(c.pen + c.s[0] + c.s[1] + c.s[2] + c.q[0] + c.q[1] + c.q[2]);
}

} // namespace box_contact
