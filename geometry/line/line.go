package line

import (
	"math"

	"github.com/downflux/orca/geometry/vector"
)

// L defines a parametric line of the form
//
//   L := P + tD
type L struct {
	p vector.V
	d vector.V
}

func New(p vector.V, d vector.V) *L { return &L{p: p, d: d} }

func (l L) P() vector.V          { return l.p }
func (l L) D() vector.V          { return l.d }
func (l L) T(t float64) vector.V { return vector.Add(l.p, vector.Scale(t, l.d)) }

// Intersect returns the t-value of a line l intersected with another line.
//
// To find the intersection point, utilize L.T().
//
// Returns error if the lines are parallel.
//
// Find the intersection between the two lines as a function of the constraint
// parameter t.
//
// Given two constraints L, M, we need to find their intersection; WLOG, let's
// project the intersection point onto L.
//
// We know the parametric equation form of these lines -- that is,
//
//   L = P + tD
//   M = Q + uE
//
// At their intersection, we know that L meets M:
//
//   L = M
//   => P + tD = Q + uE
//
// We want to find the projection onto L, which means we need to find a concrete
// value for t. the other parameter u doesn't matter so much -- let's try to get
// rid of it.
//
//   uE = P - Q + tD
//
// Here, we know P, D, Q, and E are vectors, and we can decompose these into a
// system of equations by isolating their orthogonal (e.g. horizontal and
// vertical) components.
//
//   uEx = Px - Qx + tDx
//   uEy = Py - Qy + tDy
//
// Solving for u, we get
//
//   (Px - Qx + tDx) / Ex = (Py - Qy + tDy) / Ey
//   => Ey (Px - Qx + tDx) = Ex (Py - Qy + tDy)
//
// We leave the task of simplifying the above terms as an exercise to the
// reader. Isolating t, and noting some common substitutions, we get
//
//   t = || E x (P - Q) || / || D x E ||
//
// See https://gamedev.stackexchange.com/a/44733 for more information.
func (l L) Intersect(m L, tolerance float64) (float64, bool) {
	d := vector.Determinant(l.D(), m.D())
	n := vector.Determinant(m.D(), vector.Sub(l.P(), m.P()))

	if math.Abs(d) < tolerance {
		return 0, false
	}

	return n / d, true
}

// Distance finds the distance between the line l and a point p.
//
// This is given by
//
//   d := ||D x (p - P)|| / ||D||
//
// See https://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html for
// more information.
func (l L) Distance(p vector.V) float64 {
	v := vector.Sub(p, l.P())
	return math.Abs(vector.Determinant(l.D(), v) / vector.Magnitude(l.D()))
}
