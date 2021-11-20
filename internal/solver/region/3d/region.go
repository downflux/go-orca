// Package region specifies a 2D subspace into projected 3D ambient space.
//
// It is possible for a 2D system of linear equations to be infeasible, e.g.
// when there is no region of intersection between two constraints pointing away
// from each other.
//
// However, we can "force" a solution by reframing the 2D linear programming
// question into a 3D one by adding a slack constraint, which allows us to add
// some arbitrary value to ensure all 2D constraints are satisfied. The 3D
// linear programming problem is to instead minimize the slack value necessary
// to fulfill all constraints. Geometrically, we can view the 3D system as
// extending each 2D linear constraint into the Z-axis, with a slight tilt.
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
// the line "points" towards the origin -- we see that the line of intersection
// between two planes can be modeled as the line bisecting the two 2D
// constraints, and that the parametric value in the line is the slack variable,
// or rather, a variable which correlates to a value on the Z-axis.
//
// The constraint plane intersection then, can be constructed by finding all
// such bisecting lines to the current 3D constraint plane.
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
package region

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-orca/internal/solver/region/2d"
)

type R struct {
}

func New(cs []constraint.C) *R { return nil }

func (r *R) Feasible() bool { return false }

func (r *R) Add(c constraint.C) (region.R, bool) { return region.R{}, false }
