// Package circular defines a 2D bounding constraint that limits the maximum
// length the solution vector may be.
package circular

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
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
	min, max, ok := l.IntersectCircle(hypersphere.C(m))
	if !ok {
		return segment.S{}, false
	}

	return *segment.New(l, l.T(min), l.T(max)), ok
}

// Within checks if the input vector is contained within the circle.
func (m M) Within(v vector.V) bool { return hypersphere.C(m).In(v) }

// V transforms the input vector such that the output will lie on a edge of the
// circle.
func (m M) V(v vector.V) vector.V {
	return vector.Scale(hypersphere.C(m).R(), vector.Unit(v))
}
