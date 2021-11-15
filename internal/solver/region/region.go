package region

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-orca/internal/solver/constraint"

	v2d "github.com/downflux/go-geometry/2d/vector"
)

type R struct {
	constraints []constraint.C
	infeasible  bool
}

func New(cs []constraint.C) *R {
	r := &R{}
	for _, c := range cs {
		r.Add(c)
	}
	return r
}

func (r *R) Feasible() bool { return !r.infeasible }

// Add creates a new feasible interval for the input constraint, given an
// existing set of constraints which already have been processed by the solver.
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
func (r *R) Add(c constraint.C) (segment.S, bool) {
	defer func() { r.constraints = append(r.constraints, c) }()

	if !r.Feasible() {
		return segment.S{}, r.Feasible()
	}

	l := *line.New(v2d.V(c.HP().P()), v2d.V(c.HP().N()))
	s := *segment.New(l, math.Inf(-1), math.Inf(0))
	for _, d := range r.constraints {
		m := *line.New(v2d.V(d.HP().P()), v2d.V(d.HP().N()))
		i, ok := l.Intersect(m)

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
		if hyperplane.Disjoint(hyperplane.HP(c.HP()), hyperplane.HP(d.HP())) || (!ok && !d.In(c.HP().P())) {
			r.infeasible = true
			return segment.S{}, r.Feasible()
		}

		// The new constraint fully invalidates the previous parallel
		// constraint -- we can safely ignore the old constraint in this
		// case.
		if !ok {
			continue
		}

		t := l.T(i)

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
		if d.In(c.HP().N()) {
			s = *segment.New(l, math.Max(s.TMin(), t), s.TMax())
		} else {
			s = *segment.New(l, s.TMin(), math.Min(s.TMax(), t))
		}
	}

	// A line segment of some line L is bounded by the t-value interval
	// [tmin, tmax]. If the given interval is invalid, it means that a valid
	// solution to the system does not exist.
	if s.Feasible() {
		return s, s.Feasible()
	}
	r.infeasible = true
	return segment.S{}, r.Feasible()
}
