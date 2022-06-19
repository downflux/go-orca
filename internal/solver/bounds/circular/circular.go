// Package circular defines a 2D bounding constraint that limits the maximum
// length the solution vector may be.
package circular

import (
	"math"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

// M defines a 2D circular bounding constraint.
type M hypersphere.C

func New(r float64) *M {
	s := M(*hypersphere.New(*vector.New(0, 0), r))
	return &s
}

// Bound returns the line (segment) of intersection between the circular
// constraint and the input constraint.
func (m M) Bound(c constraint.C) (segment.S, bool) {
	l := hyperplane.Line(hyperplane.HP(c))
	if hypersphere.C(m).R() == math.Inf(-1) || hypersphere.C(m).R() == math.Inf(0) {
		return *segment.New(l, math.Inf(-1), math.Inf(0)), true
	}

	v1, v2, ok := l.IntersectCircle(hypersphere.C(m))
	if !ok {
		return segment.S{}, false
	}

	t1 := l.T(v1)
	t2 := l.T(v2)

	return *segment.New(l, math.Min(t1, t2), math.Max(t1, t2)), ok
}

// Within checks if the input vector is contained within the circle.
//
// TODO(minkezhang): Rename to In instead.
func (m M) Within(v vector.V) bool {
	return hypersphere.C(m).In(v) || epsilon.Absolute(1e-5).Within(
		vector.SquaredMagnitude(vector.Sub(v, hypersphere.C(m).P())),
		hypersphere.C(m).R()*hypersphere.C(m).R(),
	)
}

// V transforms the input vector such that the output will lie on a edge of the
// circle.
func (m M) V(v vector.V) vector.V {
	r := hypersphere.C(m).R()
	if r == math.Inf(-1) || r == math.Inf(0) {
		panic("cannot map a vector to the boundary of a circle with infinite radius")
	}
	return vector.Scale(r, vector.Unit(v))
}
