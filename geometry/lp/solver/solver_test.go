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

	// Find two intersections between line and circle. This is equivalent to
	// having two additional constraints which lie tangent to the circle at
	// these two points.
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
	// TODO(minkezhang): Implment direction opitmization.

	// We represent the linear constraint as a vector
	//
	//   l := P + td
	//
	// We want to find the point on l which is closest to the agent goal
	// velocity G; this is equivalent to finding the distance between G and
	// l, and solving for t_min.
	t := vector.Dot(l, vector.Sub(r.a.G(), r.cs[i].P()))
	// If t_min lies beyond tl or tr, we know that t_min will fail to
	// satisfy at least one constraint (i.e. lies outside the boundaries of
	// at least one half-plane). The "best" we can do is to bound our
	// solution to the parametric bounds.
	if t < tl {
		result = vector.Add(r.cs[i].P(), vector.Scale(tl, l))
	} else if t > tr {
		result = vector.Add(r.cs[i].P(), vector.Scale(tr, l))
	} else {
		// If t_min lies between tl and tr, then we know "optimal" t
		// value is t_min and we can substitute directly into the
		// result.
		result = vector.Add(r.cs[i].P(), vector.Scale(t, l))
	}
	return result, true
}
