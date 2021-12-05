// Package solver solves a 2D linear programming problem projected into 3D
// ambient space.
//
// It is possible for a 2D system of linear equations to be infeasible, e.g.
// when there is no region of intersection between two constraints pointing away
// from each other.
//
// However, we can "force" a solution by reframing the 2D linear programming
// question into a 3D one by adding a slack constraint, which allows us to add
// some arbitrary value to ensure all 2D constraints are satisfied --
//
//   ax + by + cz <= b
//
// The 3D linear programming problem is to instead minimize the slack value
// necessary to fulfill all constraints. Geometrically, we see the 3D is a
// system of constraint lines on the XY-plane, but extending into the third
// dimension and tilting towards the Z-axis. The scalar associated with the
// slack variable z helps control the slope of the plane; this is useful to help
// define the lines of intersection between two planes, as seen below.
//
// From the chapter on linear programming in [1], we know that when solving a 2D
// system of linear constraints for an optimization function with a single
// solution, the iterative solution must lie on the new constraint being added.
// In our 3D analog then, we know the optimal solution must lie on the plane of
// the new 3D constraint being added in the iterative loop.
//
// Following the logic from [1], we must calculate 3D intersection of the
// constraint "plane" with all other constraints in the iteration, and then
// solve the 2D problem on this planar subspace. This is rather difficult if we
// were to solve the general problem.
//
// However, the problem becomes much simpler to solve when we decide to consider
// the problem by "flattening" down the 3D problem back into 2D space --
//
// Consider our original system of 2D constraints on the XY-plane, and extending
// into the Z-axis with a slight tilt. WLOG if we view the XY-plane from the
// directly from the positive Z-axis, the lines of intersection between planes
// are just regular 2D parametric lines, i.e.
//
//   L = P + tD
//
// Consider two completely vertical intersecting planes (i.e., their line of
// intersection is a vertical line), and then tilt the system sligtly so that
// the line "points" towards the Z-axis -- we see that the line of intersection
// between two planes can be modeled as the line projecting the two 2D
// constraints, and that the parametric value in the line is the slack variable,
// or rather, a variable which correlates to a value on the Z-axis, and the
// orientation of the intersecting line should be defined by the usual
// plane-plane intersection definition (i.e. the cross product of the two plane
// normals). We want to ensure that the line's feasible region points into the
// shared space of the two constraint planes.
//
// The constraint plane intersection then, can be constructed by finding all
// such projecting lines to the current 3D constraint plane.
//
// Remember that We want our 3D result vector to "minimally" violate some subset
// of constraints; thus, we want to provide as the target vector into the 2D
// system of equations to be the line pointing to the feasible region of the 2D
// constraint being added [2].
//
// The projected 2D problem them proceeds to effectively find the t-value which
// will minimize the distance to the target vector, i.e. the Z-axis slice that
// satisfies all current constraints and is as close as possible to the original
// infeasible slice.
//
// As a final note, we model the max speed constraint as effectively a cylinder;
// we reason that in the case the where the 2D system is infeasible, we want to
// have all solutions move as fast as possible into a feasible configuration,
// and set the speed to be the max possible, i.e. on the surface of the
// cylinder. Note that in our projected system, the distance to each constraint
// "plane" may well exceed this max speed constraint in the Z-axis, as the slack
// variable does not have a phsyical interpretation.
//
// [1]: Computational Geometry: Algorithms and Applications (de Berg et al.),
// [2]: Sidenote, the vector should technically be pointing perpendicular to the
// 3D constraint plane, but since we are projecting everything into 2D space,
// the Z-axis is removed and we can consider just the 2D constraint normal.
package solver

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/solver/2d"
)

type M interface {
	solver.M

	// V takes as input a vector and returns a vector which is guaranteed to
	// be inside the bounds of the 2D solver bounding constraint. This
	// function "seeds" the 2D solver with an initial valid solution which
	// lies on the corner of the bounding constraint.
	V(v vector.V) vector.V
}

type Unbounded struct {
	solver.Unbounded
}

func (Unbounded) V(v vector.V) vector.V { return v }

type region struct {
	m M

	// constraints is a list of the existing 2D constraints -- that is, the
	// constraints which defined an infeasible region that necessitated
	// calling the 3D solver. These constraints correspond to the "real"
	// space; in RVO, these constraints are the set of 2D ORCA lines (read:
	// hyperplanes).
	constraints []constraint.C

	infeasible bool
}

func (r *region) Feasible() bool        { return !r.infeasible }
func (r *region) Append(c constraint.C) { r.constraints = append(r.constraints, c) }

func (r *region) Solve(c constraint.C) (vector.V, bool) {
	cs, ok := r.project(c)

	if !ok {
		return vector.V{}, r.Feasible()
	}

	// Our target is to move directly into the feasible region of the
	// incremental constraint, as fast as the bounding constraints will
	// allow us -- that is, ensure that the normal vector is projected into
	// the edge of the bounding constraints.
	v := r.m.V(hyperplane.HP(c).N())

	return solver.Solve(r.m, cs, func(s segment.S) vector.V {
		// As in hyperplane.Line, we are defining the normal of a line
		// to be pointing into the feasible region of hyperplane, which
		// is defined as a vector rotated anti-clockwise to the line direction.
		c := *constraint.New(s.L().P(), s.L().N())

		// Find a t-value in the projected constraints which will
		// minimizes the distance along the Z-axis to the target vector.
		//
		// TODO(minkezhang): Verify if any of this statement is true.
		//
		// This is the optimization function which the official RVO2
		// implementation has embedded into LP1 for
		//
		//   directionOpt = true
		//
		// It is not immediately obvious how this function optimizes the
		// direction from the optimal vector to the constraint plane.
		if !c.In(v) {
			return s.L().L(s.TMin())
		}
		return s.L().L(s.TMax())
	}, v)
}

// project reduces the current 3D constraint problem into a projected 2D
// constraint problem.
func (r *region) project(c constraint.C) ([]constraint.C, bool) {
	if !r.Feasible() {
		return nil, r.Feasible()
	}

	pcs := make([]constraint.C, 0, len(r.constraints))

	for _, d := range r.constraints {
		// project takes as input two 2D linear constraints and returns
		// a new constraint which represents the line of intersection of
		// the two input constraints in 3D space. See package
		// documentation for more details on how / why we wish to do
		// this.
		var pc constraint.C

		l := hyperplane.Line(hyperplane.HP(c))
		m := hyperplane.Line(hyperplane.HP(d))

		i, ok := l.Intersect(m)

		// shared feasible region if the constraint being added
		// "relaxes" a previous parallel constraint, as the new optimal
		// solution must lie on the surface of the current incremental
		// constraint.  We need to check for this condition in the
		// caller and ensure we do not call Solve() in this case.
		if !ok && l.Parallel(m) {
			if !d.In(hyperplane.HP(c).P()) {
				r.infeasible = true
				return nil, r.Feasible()
			}
			// The incremental constraint "tightens" the previous
			// constraint and fully invalidates it.
			pc = c
		} else if !ok && !l.Parallel(m) {
			// The two constraints are anti-parallel.
			//
			// The distance ratio between this new constraint and
			// the constraints C and D can be is the assumption all
			// agents act symmetrically. When calculating
			// constraints for inflexible walls, we will need to
			// offset this new plane accordingly.
			pc = *constraint.New(
				vector.Scale(0.5, vector.Add(l.P(), m.P())),
				hyperplane.HP(c).N(),
			)
		} else {
			// Just as in the 2D case, we do not consider there to be a
			// The two constraints intersect.
			pc = *constraint.New(
				i,
				// We want the line of intersection to bisect
				// the constraints, so we need to ensure the two
				// input vectors have equal "weight".
				//
				// The ratio of angles between this new
				// constraint and the constraints C and D can be
				// is the assumption all agents act
				// symmetrically. When calculating constraints
				// for inflexible walls, we will need to offset
				// this new plane accordingly.
				vector.Unit(
					vector.Sub(
						vector.Unit(hyperplane.HP(d).N()),
						vector.Unit(hyperplane.HP(c).N()),
					),
				),
			)
		}

		pcs = append(pcs, pc)
	}

	return pcs, true
}

// Solve calculates a solution to an infeasible 2D linear programming problem by
// adding a slack variable, i.e. adding a third dimension.
//
// N.B: This is not a general-purpose 3D linear programming solver. Both the
// bounding constraints M and input constraints are 2D-specific.
func Solve(m M, cs []constraint.C, v vector.V) vector.V {
	if !m.Within(v) {
		return vector.V{}
	}

	// dist is the current penetration distance into the infeasible region
	// of some constraint plane from the input.
	dist := 0.

	r := &region{
		m:           m,
		constraints: make([]constraint.C, 0, len(cs)),
	}
	for _, c := range cs {
		l := hyperplane.Line(hyperplane.HP(c))
		// The base 2D linear programming problem may be infeasible. In
		// order to "solve" this problem, we are systematically relaxing
		// the 2D constraint requirements with a slack variable
		// (represented by the penetration distance). That is, we accept
		// the solution to Solve() may lie in the infeasible region of
		// some subset of input 2D constraints, but we want to minimize
		// this distance across all such constraints.
		//
		// If the penetration distance to the iterative constraint
		// exceeds the current minimal value of the slack, then we need
		// to find a new minimum. Note that this new value of the slack
		// will exceed the old distance as well -- but it may be smaller
		// than the current solution we have found.
		if !c.In(v) && l.Distance(v) > dist {
			// In the case r.Solve() returns infeasible due to a
			// rounding error, we ignore the result and continue.
			if u, ok := r.Solve(c); ok {
				v = u
			}
		}

		r.Append(c)
		dist = l.Distance(v)
	}
	return v
}
