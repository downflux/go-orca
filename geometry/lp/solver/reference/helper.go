package reference

import (
	"math"

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

const (
	epsilon = 1e-10
)

type Agent interface {
	S() float64
	T() vector.V
}

// Helper implements linearProgram1 from the reference RVO2
// implmentation.
type Helper struct {
	cs []plane.HP
	a  Agent
}

// Add calculates a vector along the constraint that minimizes the distance to
// the preferred velocity.
//
// Note that while a constraint may be defined in such a way that the preferred
// vector lies inside it, we do not return the preferred vector in that case;
// while the intersection of constraints represent feasible velocities which
// will allow an agent to avoid all other agents, the constraint-axis
// intersection (i.e. P in C := P + tD) represents the optimal velocity between
// colliding agents; this implies when we move off of the constraint line, we
// are in fact violating our agreement with neighboring agents, negatively
// impacting our overall group consensus.
//
// TODO(minkezhang): Think more about this and migrate to ORCA doc.
func (r Helper) Add(constraint plane.HP) (vector.V, bool) {
	dot := vector.Dot(constraint.P(), constraint.D())
	discriminant := dot*dot + r.a.S()*r.a.S() - vector.SquaredMagnitude(constraint.P())

	if discriminant < 0 {
		return vector.V{}, false
	}

	// Find two intersections between line and circle. This is equivalent to
	// having two additional constraints which lie tangent to the circle at
	// these two points.
	tl := -dot - math.Sqrt(discriminant)
	tr := -dot + math.Sqrt(discriminant)

	// TODO(minkezhang): Make a parametric.LImpl struct implementing
	//
	// type L interface {
	//   T(float)     vector.V  // (x, y) coordinate for a given t value.
	//   Intersect(L) float     // L1 t-value at which L1 and L2 intersect.
	//   P()          vector.V  // Origin point of the line.
	//   D()          vector.V  // Direction of the line.
	// }

	for _, c := range r.cs {
		d := vector.Determinant(c.D(), constraint.D())
		n := vector.Determinant(constraint.D(), vector.Sub(c.P(), constraint.P()))
		if d < epsilon {
			if n < 0 {
				return vector.V{}, false
			}
			continue
		}

		// Find the intersection between the two lines as a function of
		// the constraint parameter t.
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
	// TODO(minkezhang): Implment direction opitmization.

	// We represent the linear constraint as a vector
	//
	//   l := P + td
	//
	// We want to find the point on l which is closest to the agent goal
	// velocity G; this is equivalent to finding the distance between G and
	// l, and solving for t_min.
	t := vector.Dot(constraint.D(), vector.Sub(r.a.T(), constraint.P()))

	// If t_min lies beyond tl or tr, we know that t_min will fail to
	// satisfy at least one constraint (i.e. lies outside the boundaries of
	// at least one half-plane). The "best" we can do is to bound our
	// solution to the parametric bounds.
	//
	// If t_min lies between tl and tr, then we know "optimal" t value is
	// t_min and we can substitute directly into the result.
	if t < tl {
		t = tl
	} else if t > tr {
		t = tr
	}

	r.cs = append(r.cs, constraint)
	return vector.Add(constraint.P(), vector.Scale(t, constraint.D())), true
}
