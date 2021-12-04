package solver

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/solver/bounds/circular"

	s2d "github.com/downflux/go-orca/internal/solver/2d"
	s3d "github.com/downflux/go-orca/internal/solver/3d"
)

// project finds the point on the input line segment which is closest to the
// input vector.
//
// Note that s.T(v) finds the projected parametric t-value of the underlying
// line, bounded by the min / max values of the line segment.
func project(s segment.S, v vector.V) vector.V {
	return s.L().L(s.T(v))
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v, with maximum length of v set to
// r.
func Solve(cs []constraint.C, v vector.V, r float64) vector.V {
	m := *circular.New(r)

	res, ok := s2d.Solve(
		m,
		cs,
		func(s segment.S) vector.V {
			return project(s, v)
		},
		v,
	)

	if !ok {
		res = s3d.Solve(m, cs, v)
	}

	return res
}
