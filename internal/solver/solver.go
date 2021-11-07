package solver

import (
	"math"

	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/constraint"
)

type S struct {
	cs        []constraint.C
	tolerance float64
}

func New(cs []constraint.C, tolerance float64) *S {
	return &S{
		cs:        cs,
		tolerance: tolerance,
	}
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v vector.V) vector.V {
	res := v

	for i, c := range s.cs {
		if !c.In(res) {
			tmin := math.Inf(-1)
			tmax := math.Inf(0)

			for j := 0; j < i; j++ {
				// TODO(minkezhang): Check for disjoint planes.

				v, ok := c.Intersect(s.cs[j].HP.L, s.tolerance)

				if !ok {
					// TODO(minkezhang): Check for parallel lines.
				}

				t := c.Project(v)

				// If a valid value in the new constraint is
				// also valid in an existing constraint, then t
				// is an upper bound on the the feasible region.
				//
				// We are iteratively finding the segment of the
				// new constraint which lies on the intersecting
				// convex polygon.
				if vector.Determinant(s.cs[j].D(), c.D()) > 0 {
					tmax = math.Min(tmax, t)
				} else {
					tmin = math.Max(tmin, t)
				}
			}

			// A line segment of some line L is bounded by the
			// t-value interval [tmin, tmax]. If the given interval
			// is invalid, it means that a valid solution to the
			// system does not exist.
			if tmin > tmax {
				// TODO(minkezhang): Implement this.
			}

			t := c.Project(v)
			if t < tmin {
				t = tmin
			} else if t > tmax {
				t = tmax
			}
			res = vector.Add(c.P(), vector.Scale(t, c.D()))
		}
	}

	return res
}
