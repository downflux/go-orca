package solver

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-orca/internal/solver/constraint"

	v2d "github.com/downflux/go-geometry/2d/vector"
)

type S struct {
	cs []constraint.C
}

func New(cs []constraint.C) *S {
	return &S{
		cs: cs,
	}
}

/*
func segment(c constraint.C, cs []constraint.C) (segment.S, bool) {
	if len(cs) == 0 {
		return *segment.New(c.L(), math.Inf(-1), math.Inf(0))
	}

	d := cs[len(cs) - 1]

	v, ok := c.HP().L().Intersect(s.cs[j].HP().L(), s.tolerance)
}
*/

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v v2d.V) v2d.V {
	res := v

	for i, c := range s.cs {
		if !c.In(vector.V(res)) {
			l := *line.New(
				v2d.V(c.HP().P()),
				v2d.V(c.HP().N()),
			)

			seg := *segment.New(l, math.Inf(-1), math.Inf(0))

			for j := 0; j < i; j++ {
				m := *line.New(
					v2d.V(s.cs[j].HP().P()),
					v2d.V(s.cs[j].HP().N()),
				)
				v, ok := l.Intersect(m)

				// Check for disjoint planes.
				//
				// If the two planes are disjoint, or the new
				// contraint "relaxes" the previous constraint,
				// we can no longer find a point on the current
				// constraint which satisfies all previous
				// constraints.
				//
				// Note that we should never call this loop to
				// relax parallel lines -- the previous
				// feasibility check should avoid the call.
				//
				// TODO(minkezhang): Throw error in this case.
				if hyperplane.Disjoint(hyperplane.HP(c.HP()), hyperplane.HP(s.cs[j].HP())) || !ok && !s.cs[j].In(c.HP().P()) {
				}

				// The new constraint fully invalidates the
				// previous parallel constraint -- we can safely
				// ignore the old constraint in this case.
				if !ok {
					continue
				}

				t := l.T(v)

				// If a valid value in the new constraint is
				// also valid in an existing constraint, then t
				// is an upper bound on the the feasible region.
				//
				// We are iteratively finding the segment of the
				// new constraint which lies on the intersecting
				// convex polygon.
				if v2d.Determinant(v2d.V(s.cs[j].HP().N()), v2d.V(c.HP().N())) > 0 {
					seg = *segment.New(l, math.Min(seg.TMax(), t), seg.TMin())
				} else {
					seg = *segment.New(l, seg.TMax(), math.Max(seg.TMin(), t))
				}
			}

			// A line segment of some line L is bounded by the
			// t-value interval [tmin, tmax]. If the given interval
			// is invalid, it means that a valid solution to the
			// system does not exist.
			//
			// TODO(minkezhang): Throw error.
			if !seg.Feasible() {
			}

			res = v2d.Add(
				v2d.V(c.HP().P()),
				v2d.Scale(
					seg.T(v),
					v2d.V(c.HP().N()),
				),
			)
		}
	}

	return res
}
