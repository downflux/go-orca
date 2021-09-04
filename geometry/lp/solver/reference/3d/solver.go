package solver

import (
	"math"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"

	s2d "github.com/downflux/orca/geometry/lp/solver/reference/2d"
)

const (
	tolerance = 1e-10
)

type S struct{}

func (s S) Solve(a helper.Agent, cs []plane.HP) (vector.V, bool) {
	var distance float64
	var solution vector.V
	helper := s2d.S{}

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
		if vector.Determinant(c.D(), vector.Sub(c.P(), solution)) < distance {
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
				ncs = append(ncs, *plane.New(p, vector.Unit(vector.Sub(d.N(), c.N()))))
			}
			// TODO(minkezhang): Need to set the preferred velocity a.T() to
			//
			//   a.T() = *vector.New(c.D().Y(), -c.D().X())
			//
			// Which is the "invalid" side of the constraint c.
			//
			// TODO(minkezhang): Implement optimizeDirection arg for
			// LP2() and LP1().
			solution, ok := helper.Solve(a, ncs)
			if !ok {
				return vector.V{}, false
			}
			distance = vector.Determinant(c.D(), vector.Sub(c.P(), solution))
		}
	}

	return solution, true
}
