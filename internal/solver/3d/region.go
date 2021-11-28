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
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/solver/2d"
)

type region struct {
	m solver.M
	o solver.O

	constraints []constraint.C
	infeasible  bool
}

func New(cs []constraint.C) *region { return &region{constraints: cs} }

func (r *region) Feasible() bool { return !r.infeasible }

func (r *region) Add(c constraint.C) (vector.V, bool) {
	defer func() { r.constraints = append(r.constraints, c) }()

	_, ok := r.project(c)
	if !ok {
		return vector.V{}, r.Feasible()
	}

	// TODO(minkezhang): Implement Add()
	return vector.V{}, false
}

// project reduces the current 3D constraint problem into a projected 2D
// constraint problem.
func (r *region) project(c constraint.C) ([]constraint.C, bool) {
	defer func() { r.constraints = append(r.constraints, c) }()

	pcs := make([]constraint.C, 0, len(r.constraints))

	for _, d := range r.constraints {
		pc, ok := project(c, d)
		if !ok {
			r.infeasible = true
			return nil, r.Feasible()
		}

		pcs = append(pcs, pc)
	}

	return pcs, true
}

// project takes as input two 2D linear constraints and returns a new constraint
// which represents the line of intersection of the two input constraints in 3D
// space. See package documentation for more details on how / why we wish to do
// this.
func project(incremental constraint.C, existing constraint.C) (constraint.C, bool) {
	l := hyperplane.Line(hyperplane.HP(incremental))
	m := hyperplane.Line(hyperplane.HP(existing))

	i, ok := l.Intersect(m)

	// Just as in the 2D case, we do not consider there to be a shared
	// feasible region if the constraint being added "relaxes" a previous
	// parallel constraint, as the new optimal solution must lie on the
	// surface of the current incremental constraint.  We need to check for
	// this condition in the caller and ensure we do not call Add() in this
	// case.
	if !ok && l.Parallel(m) {
		if !existing.In(hyperplane.HP(incremental).P()) {
			return constraint.C{}, false
		}
		// The incremental constraint "tightens" the previous constraint
		// and fully invalidates it.
		return incremental, true
	} else if !ok && !l.Parallel(m) {
		// The two constraints are anti-parallel.
		return *constraint.New(
			vector.Scale(0.5, vector.Sub(l.P(), m.P())),
			hyperplane.HP(incremental).N(),
		), true
	}
	// The two constraints intersect.
	return *constraint.New(
		i,
		vector.Sub(
			hyperplane.HP(existing).N(),
			hyperplane.HP(incremental).N(),
		),
	), true
}
