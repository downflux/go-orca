// Package region specifies a 2D subspace in 2D ambient space.
package region

import (
	"math"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

// O specifies an optimization function for the 2D lineaer problem. This
// function assumes there a single optimal value found over the span of the
// segment. Note that the function may not be linear -- this is a relaxation on
// the general LP problem.
//
// For example, we may use the distance function as a optimization function,
// even though the function itself is not of degree 1, because the distance
// function has a single root.
type O func(s segment.S) vector.V

// M is an additional (potential non-linear) constraint which clamps the
// solution a minimum and maximum value. This corresponds to the bounded
// constraints defined in de Berg et al. (2008). Note that while de Berg assumes
// all constraints are linear, we relax this assumption slightly and effectively
// allow specifying per-constraint bounds.
//
// This is useful for e.g. when the bounded constraint is circular, as in the
// case of solving an LP problem in velocity-space with a maximum speed
// constraint. We can model the circular constraint as a series of linear
// constraints which lie tangent to the circle-constraint intersection. To make
// this slightly more efficient, we can instead just truncate the intersection
// of each "real" constraint to be fully bounded by the circle-constraint
// intersection.
//
// M will return false if the input constraint lies outside the bounds of M.
type M func(c constraint.C) (segment.S, bool)

// Unbounded is the null constraint.
func Unbounded(c constraint.C) (segment.S, bool) {
	l := hyperplane.Line(hyperplane.HP(c))
	return *segment.New(l, math.Inf(-1), math.Inf(0)), true
}

type R struct {
	m           M
	o           O
	constraints []constraint.C
	infeasible  bool
}

// New constructs a new 2D region.
func New(m M, o O) *R {
	return &R{
		m: m,
		o: o,
	}
}

func (r *R) Feasible() bool { return !r.infeasible }

// Add appends the given constraint into the region and returns the optimal
// value along the interval intersection. The optimal value is calculated based
// on the optimization function given to the region at construction time.
//
// This is analogous to Agent.linearProgram1 in the official RVO2
// implementation.
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
// N.B.: This function is not order-invariant, especially notable in the case
// where the new constraint is parallel to and contains points external to an
// existing constraint. The calling function is responsible for ensuring this
// function is not called for this specific edge case by e.g. checking for when
// the iterative optimal solution is already feasible for the new constraint.
func (r *R) Add(c constraint.C) (vector.V, bool) {
	defer func() { r.constraints = append(r.constraints, c) }()

	s, ok := r.intersect(c)
	if !ok {
		return vector.V{}, r.Feasible()
	}

	return r.o(s), r.Feasible()
}

// intersect creates a new feasible interval for the input constraint, given an
// existing set of constraints which already have been processed by the solver.
func (r *R) intersect(c constraint.C) (segment.S, bool) {
	if !r.Feasible() {
		return segment.S{}, r.Feasible()
	}

	s, ok := r.m(c)
	if !ok {
		r.infeasible = true
		return segment.S{}, r.Feasible()
	}

	l := hyperplane.Line(hyperplane.HP(c))
	for _, d := range r.constraints {
		i, ok := l.Intersect(
			hyperplane.Line(hyperplane.HP(d)),
		)

		// Check for disjoint planes.
		//
		// If the two planes are disjoint, or the new contraint is
		// parallel to but "relaxes" the previous constraint, we can no
		// longer find a point on the current constraint which satisfies
		// all previous constraints.
		//
		// Note that we should never call this loop to relax parallel
		// lines -- the previous feasibility check should avoid the
		// call.
		if hyperplane.Disjoint(
			hyperplane.HP(c),
			hyperplane.HP(d),
		) || (!ok && !d.In(hyperplane.HP(c).P())) {
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
		// C.L(t) ∈ F(D) ⇒ C.L(t + 𝛿) ∈ F(D)
		//
		// where F(D) is the feasible region of D.
		//
		// Note that we can make a vector out of the two points C.L(t)
		// and C.L(t + 𝛿); by orienting this vector towards C.L(t + 𝛿),
		// and noting that this is just C.D(), we can check the validity
		// of the implication statment above by checking the feasibility
		// of C.D() in the F(D) directly.
		//
		// Should C.D() lie in F(D), this means all points on the line
		// with parametric t-values greater than our intersection value
		// t also lie in F(D) -- and thus, t is a lower bound for the
		// feasibility segment.
		if d.In(l.D()) {
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
