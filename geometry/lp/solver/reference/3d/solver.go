package solver

import (
	"fmt"
	"math"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"

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
	var distance float64
	solution := a.T()
	helper := *s2d.New(true)

	for i, c := range cs {
		// Since RVO2 line direction D is anti-parallel to our
		// definition of D, we must flip the inequality check.
		//
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
		// N.B.: RVO2 defines the distance vector as pointing into the
		// plane; we are using the less confusing orientation of
		// pointing our distance vector towards the solution, which
		// causes us to flip the inequality.
		fmt.Println("DEBUG: Solve() checking constraint c.P() == ", c.P(), "c.D() == ", c.D())
		fmt.Println("DEBUG: Solve() ", vector.Determinant(c.D(), vector.Sub(c.P(), solution)))
		if vector.Determinant(c.D(), vector.Sub(c.P(), solution)) > distance {
			fmt.Println("DEBUG: Solve() optimizing solution = ", solution)
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
			// TODO(minkezhang): Implement optimizeDirection arg for
			// LP2() and LP1().
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
