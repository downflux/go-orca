package solver

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/solver/region"
)

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
	res, ok := optimize(v, s.cs)
	if !ok {
		res = expand(s.cs)
	}

	return res
}

// TODO(minkezhang): Implement LP3.
func expand(cs []constraint.C) vector.V { return vector.V{} }

// optimize attempts to calculate a new vector which is as close to the
// input vector as possible while satisfying all ORCA half-planes. If there is
// no shared region between all half-planes, this function will return
// infeasible.
//
// The order by which constraints are given to this function does not matter; we
// are adding the constraints iteratively, and refining our solution similar to
// the simplex approach, though applied to a non-linear constraint due to the
// distance optimization target.
//
// This is analogous to Agent.linearProgram2 in the official RVO2
// implementation.
//
// TODO(minkezhang): Add callback function for how to iteratively set the result
// vector after getting back a segment.
func optimize(v vector.V, cs []constraint.C) (vector.V, bool) {
	r := region.New(nil)

	res := v
	for _, c := range cs {
		if !c.In(res) {
			s, ok := r.Add(c)
			if !ok {
				return vector.V{}, false
			}

			// Find the projected parametric t-value which minimizes
			// the distance to the optimal vector. If the t-value
			// lies outside the line segment, use the nearest
			// segment boundary value instead.
			t := s.T(v)

			res = hyperplane.Line(hyperplane.HP(c)).L(t)
		}
	}
	return res, true
}
