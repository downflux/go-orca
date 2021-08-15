package solver

import (
	"math"

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"

	agent "github.com/downflux/orca/vo/agent/reference"
)

const (
	tolerance = 1e-10
)

// ReferenceHelper implements linearProgram1 from the reference RVO2
// implmentation.
type ReferenceHelper struct {
	cs []plane.HP
	a  agent.A
}

func (r ReferenceHelper) Solve(i int) (vector.V, bool) {
	result := vector.V{}

	l := *vector.New(-r.cs[i].N().Y(), r.cs[i].N().X())
	dot := vector.Dot(r.cs[i].P(), l)
	discriminant := dot*dot + r.a.R()*r.a.R() - vector.SquaredMagnitude(r.cs[i].P())

	if discriminant < 0 {
		return vector.V{}, false
	}

	// Find two intersections between line and circle.
	tl := -dot - math.Sqrt(discriminant)
	tr := -dot + math.Sqrt(discriminant)

	for j, c := range r.cs {
		if j < i {
			lj := *vector.New(-c.N().Y(), c.N().X())
			d := vector.Determinant(lj, l)
			n := vector.Determinant(l, vector.Sub(c.P(), r.cs[i].P()))
			if d < tolerance {
				if n < 0 {
					return vector.V{}, false
				}
				continue
			}

			t := n / d
			if d > 0 {
				// tl and tr is mutated across loop boundaries.
				//
				// TODO(minkezhang): Reason out that this is the
				// "memory" of the optimal result.
				tr = math.Min(tr, t)
			} else {
				tl = math.Max(tl, t)
			}

			if tl > tr {
				return vector.V{}, false
			}
		}
	}
	// Skip direction opitmization for now.

	// Returns a result vector which is closest to v_pref.
	t := vector.Dot(l, vector.Sub(r.a.G(), r.cs[i].P()))
	if t < tl {
		// P = E + tD
		result = vector.Add(r.cs[i].P(), vector.Scale(tl, l))
	} else if t > tr {
		result = vector.Add(r.cs[i].P(), vector.Scale(tr, l))
	} else {
		// Optimal point is on the circle (?)
		//
		// TODO(minkezhang): Verify this is actually the
		// reasoning.
		result = vector.Add(r.cs[i].P(), vector.Scale(t, l))
	}
	return result, true
}
