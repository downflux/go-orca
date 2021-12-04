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

// S implements a linear program solver for a system of 2D constraints. In the
// case of an infeasible region, the solver will relax the constraints until a
// single optimal solution may be found.
type S struct {
	cs []constraint.C
	r  float64
}

func New(cs []constraint.C) *S {
	return &S{
		cs: cs,
	}
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v vector.V) vector.V {
	res, ok := s2d.Solve(
		s2d.Unbounded{},
		s.cs,
		func(s segment.S) vector.V {
			return project(s, v)
		},
		v,
	)

	if !ok {
		res = s3d.Solve(*circular.New(s.r), s.cs, v)
	}

	return res
}
