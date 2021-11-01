package line

import (
	"math"

	"github.com/downflux/orca/geometry/circle"
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
//
// TODO(minkezhang): Return a vector of the intersection point instead.
func (l L) Intersect(m L, tolerance float64) (float64, bool) {
	d := vector.Determinant(l.D(), m.D())
	n := vector.Determinant(m.D(), vector.Sub(l.P(), m.P()))

	if math.Abs(d) < tolerance {
		return 0, false
	}

	return n / d, true
}

// IntersectCircle returns the t-values at which the line intersects a circle.
// If the line does not intersect a circle, the function will return not
// successful.
//
// As a line stretches to infinity in both directions, it is not possible for a
// line to intersect the circle partway.
//
// If the line lies tangent to the circle, then the returned t-values are the
// same.
//
// See https://stackoverflow.com/a/1084899 for more information.
//
// TODO(minkezhang): Return a vector tuple of the intersection points instead.
func (l L) IntersectCircle(c circle.C) (float64, float64, bool) {
	p := vector.Sub(l.P(), c.P())

	dot := vector.Dot(p, l.D())
	discriminant := dot*dot + c.R()*c.R() - vector.SquaredMagnitude(p)

	// The line does not intersect the circle.
	if discriminant < 0 {
		return 0, 0, false
	}

	// Find two intersections between line and circle. This is equivalent to
	// having two additional constraints which lie tangent to the circle at
	// these two points.
	tl := -dot - math.Sqrt(discriminant)
	tr := -dot + math.Sqrt(discriminant)

	return tl, tr, true
}

// Distance finds the distance between the line l and a point p.
//
// The distance from a line L to a point Q is given by
//
//   d := || D x (Q - P) || / || D ||
//
// See
// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
// for more information.
func (l L) Distance(p vector.V) float64 {
	v := vector.Sub(p, l.P())
	return math.Abs(vector.Determinant(l.D(), v) / vector.Magnitude(l.D()))
}
