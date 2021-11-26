package solver

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"

	r2d "github.com/downflux/go-orca/internal/solver/region/2d"
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
}

func New(cs []constraint.C) *S {
	return &S{
		cs: cs,
	}
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v vector.V) vector.V {
	res, ok := solve2D(v, s.cs)
	if !ok {
		res = solve3D(s.cs)
	}

	return res
}

// TODO(minkezhang): Implement LP3.
func solve3D(cs []constraint.C) vector.V { return vector.V{} }

// solve2D attempts to calculate a new vector which is as close to the input
// vector as possible while satisfying all ORCA half-planes. If there is no
// shared region between all half-planes, this function will return infeasible.
//
// The order by which constraints are given to this function does not matter; we
// are adding the constraints iteratively, and refining our solution similar to
// the simplex approach, though applied to a non-linear constraint due to the
// distance optimization target.
//
// This is analogous to Agent.linearProgram2 in the official RVO2
// implementation.
func solve2D(v vector.V, cs []constraint.C) (vector.V, bool) {
	r := r2d.New(
		r2d.Unbounded,
		func(s segment.S) vector.V {
			return project(s, v)
		},
	)

	res := v
	for _, c := range cs {
		var ok bool
		if !c.In(res) {
			if res, ok = r.Add(c); !ok {
				return vector.V{}, false
			}
		}
	}
	return res, true
}
