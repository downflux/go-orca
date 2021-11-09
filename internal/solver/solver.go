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
	res, ok := generateResult(v, s.cs, s.tolerance)
	if !ok {
		panic("Cannot generate a feasible result with the given constraints; need to implement the 3D solver.")
	}

	return res
}

// generateResult attempts to calculate a new vector which is as close to the
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
func generateResult(v vector.V, cs []constraint.C, tolerance float64) (vector.V, bool) {
	res := v
	for i, c := range cs {
		if !c.In(res) {
			seg, ok := generateSegment(c, cs[:i], tolerance)
			if !ok {
				return vector.V{}, false
			}

			// Find the projected parametric t-value which minimizes
			// the distance to the optimal vector. If the t-value
			// lies outside the line segment, use the nearest
			// segment boundary value instead.
			t := seg.Project(v)

			res = c.HP().L().T(t)
		}
	}
	return res, true
}

// generateSegment creates a new feasible interval for the input constraint,
// given an existing set of constraints which already have been processed by the
// solver.
//
// This function is called when the new constraint C invalidates the iterative
// solution -- that is, the current solution lies outside the valid region of C,
// and WLOG so too does the target minimization vector. Because this solver
// attempts to find a solution a minimal distance away from the target vector,
// we know that any returned solution must lie on the new constraint line
// itself, and not further away in the feasible zone; thus, this solution works
// with the characteristic line of C for the majority of the computing logic
// here.
//
// This is analogous to Agent.linearProgram1 in the official RVO2
// implementation.
//
// N.B.: This function is not order-invariant, especially notable in the case
// where the new constraint is parallel to and contains points external to an
// existing constraint. The calling function is responsible for ensuring this
// function is not called for this specific edge case by e.g. checking for when
// the iterative optimal solution is already feasible for the new constraint.
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
