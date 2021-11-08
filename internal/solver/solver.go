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
		if plane.Disjoint(c.HP(), d.HP(), tolerance) || (!ok && !d.In(c.HP().P())) {
			return segment.S{}, false
		}

		// The new constraint fully invalidates the previous parallel
		// constraint -- we can safely ignore the old constraint in this
		// case.
		if !ok {
			continue
		}

		t := c.HP().L().Project(i)

		// We are iteratively finding the segment of the new constraint
		// which lies on the intersecting convex polygon.
		//
		// Consider the new constraint being added C, and the current
		// looped constraint variable D; we wish to determine if the
		// projected parametric t-value onto C of the C-D intersection
		// should be used to refine our lower or upper bound for the
		// feasible line segment.
		//
		// By definition, the feasible line segment of C satisfies all
		// previous constraints, including D -- we wish to see if
		//
		// C.T(t) ∈ F(D) ⇒ C.T(t + 𝛿) ∈ F(D)
		//
		// where F(D) is the feasible domain of D.
		//
		// Note that we can make a vector out of the two points C.T(t)
		// and C.T(t + 𝛿); by orienting this vector towards C.T(t + 𝛿),
		// and noting that this is just C.D(), we can check the validity
		// of the implication statment above by checking the feasibility
		// of C.D() in the F(D) directly.
		//
		// Should C.D() lie in F(D), this means all points on the line
		// with parametric t-values greater than our intersection value
		// t also lie in F(D) -- and thus, t is a lower bound for the
		// feasibility segment.
		if d.In(c.HP().D()) {
			s = *segment.New(c.HP().L(), math.Max(s.TMin(), t), s.TMax())
		} else {
			s = *segment.New(c.HP().L(), s.TMin(), math.Min(s.TMax(), t))
		}
	}

	// A line segment of some line L is bounded by the t-value interval
	// [tmin, tmax]. If the given interval is invalid, it means that a valid
	// solution to the system does not exist.
	return s, s.Feasible()
}
