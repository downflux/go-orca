package solver

import (
	"math"

	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/segment"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/solver/constraint"
)

type S struct {
	cs        []constraint.C
	tolerance float64
}

func New(cs []constraint.C, tolerance float64) *S {
	return &S{
		cs:        cs,
		tolerance: tolerance,
	}
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v vector.V) vector.V {
	res := v

	for i, c := range s.cs {
		if !c.In(res) {
			seg, ok := generateSegment(c, s.cs[:i], s.tolerance)

			// TODO(minkezhang): Throw error.
			if !ok {
			}

			// Find the projected parametric t-value which minimizes
			// the distance to the optimal vector. If the t-value
			// lies outside the line segment, return the segment
			// boundary value instead.
			t := seg.Project(v)

			res = c.HP().L().T(t)
		}
	}

	return res
}

// generateSegment creates a new feasible interval for the given constraint,
// given an existing set of constraints which already have been processed by the
// solver.
//
// N.B.: This function is not order-invariant, specifically in the case where
// the new constraint is parallel and contains points external to an existing
// constraint. The calling function should make sure to not call this function
// in these cases, i.e. when the iterative optimal solution is alerady feasible
// for the new constraint.
func generateSegment(c constraint.C, cs []constraint.C, tolerance float64) (segment.S, bool) {
	s := *segment.New(c.HP().L(), math.Inf(-1), math.Inf(0))

	for _, d := range cs {
		i, ok := c.HP().L().Intersect(d.HP().L(), tolerance)

		// Check for disjoint planes.
		//
		// If the two planes are disjoint, or the new contraint
		// "relaxes" the previous constraint, we can no longer find a
		// point on the current constraint which satisfies all previous
		// constraints.
		//
		// Note that we should never call this loop to relax parallel
		// lines -- the previous feasibility check should avoid the
		// call.
		if plane.Disjoint(c.HP(), d.HP(), tolerance) || !ok && !d.In(c.HP().P()) {
			return s, false
		}

		// The new constraint fully invalidates the previous parallel
		// constraint -- we can safely ignore the old constraint in this
		// case.
		if !ok {
			continue
		}

		t := c.HP().L().Project(i)

		// If a valid value in the new constraint is also valid in an
		// existing constraint, then t is an upper bound on the the
		// feasible region.
		//
		// We are iteratively finding the segment of the new constraint
		// which lies on the intersecting convex polygon.
		if vector.Determinant(d.HP().D(), c.HP().D()) > 0 {
			s = *segment.New(c.HP().L(), math.Min(s.TMax(), t), s.TMin())
		} else {
			s = *segment.New(c.HP().L(), s.TMax(), math.Max(s.TMin(), t))
		}
	}

	// A line segment of some line L is bounded by the t-value interval
	// [tmin, tmax]. If the given interval is invalid, it means that a valid
	// solution to the system does not exist.
	return s, s.Feasible()
}
