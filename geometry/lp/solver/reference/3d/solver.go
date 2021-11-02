package solver

import (
	"math"

	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/orca/geometry/lp/solver/reference/helper"

	s2d "github.com/downflux/orca/geometry/lp/solver/reference/2d"
)

const (
	tolerance = 1e-10
)

type A struct {
	r float64
	t vector.V
}

func (a A) S() float64  { return a.r }
func (a A) T() vector.V { return a.t }

type S struct{}

func (s S) Solve(a helper.Agent, cs []plane.HP) (vector.V, bool) {
	// distance represents a solution optimization factor; when the distance
	// between an existing solution and a new constraint exceeds this metric, we should
	var distance float64

	// N.B.: a.T() is in velocity space, as are the contraints and velocity
	// objects.
	solution := a.T()
	helper := *s2d.New(true)

	for i, c := range cs {
		// The distance between a point Q and a line L is given by
		//
		//   d := || (P - Q) x D || / || D ||
		//
		// Where L is of the form
		//
		//   L := P + tD
		//
		// See
		// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
		// for more information.
		//
		// The returned solution may not be feasible for all
		// constraints; however, we want to ensure that the solution
		// minimizes the penetration into the forbidden region. Given a
		// solution which is of distance d from a list of constraints,
		// we will skip recalculating d if it is already feasible for
		// the next contraint C, and if the distance between the
		// solution and C is less than the existing minimal criteria.
		//
		// Note that the sequence of optimization distances is not
		// monotonically decreasing -- we are optimizing the direction
		// of the solution in LP2, which may in fact increase the
		// solution distance.
		//
		// N.B.: RVO2 constraint infeasibility zones are to the right of
		// a line L; we have defined the infeasibility zone to the left.
		// However, we have also flipped the test vector here from P - Q
		// to Q - P, which allows us to flip the inequality again, such
		// that our final inequality check aligns with the RVO2
		// implementation.
		if vector.Determinant(c.D(), vector.Sub(solution, c.P())) > distance {
			var ncs []plane.HP

			for _, d := range cs[:i] {
				var p vector.V

				determinant := vector.Determinant(c.D(), d.D())
				if math.Abs(determinant) <= tolerance {
					// Constraints c and d are oriented in the same direction.
					if vector.Dot(c.D(), d.D()) > 0 {
						continue
					}
					p = vector.Scale(0.5, vector.Add(c.P(), d.P()))
				} else {
					t := vector.Determinant(d.D(), vector.Sub(c.P(), d.P())) / determinant
					p = vector.Add(c.P(), vector.Scale(t, c.D()))
				}
				// Note the direction of the newly constructed
				// contraint is not "in between" the two
				// constraints c and d.
				ncs = append(ncs, *plane.New(p, vector.Unit(vector.Sub(d.N(), c.N()))))
			}
			var ok bool
			// We are trying to "expand" the intersected region
			// outwards to find a valid solution; thus, we are
			// trying to optimize for a target
			//
			//   T := *vector.New(c.D().Y(), -c.D().X())
			//
			// pointing to the "invalid" side of the constraint c.
			//
			// Any solution to this problem will return a vector
			// which minimizes the distance between the set of
			// constraints and this rotated target.
			//
			// Note that we are optimizing direction here -- this
			// may cause us to actually increase the minimal
			// distance. However, we are guaranteed that the
			// returned result is valid for all constraints.
			solution, ok = helper.Solve(A{
				r: a.S(),
				t: *vector.New(c.D().Y(), -c.D().X()),
			}, ncs)
			if !ok {
				return vector.V{}, false
			}
			distance = vector.Determinant(c.D(), vector.Sub(solution, c.P()))
		}
	}

	return solution, true
}
