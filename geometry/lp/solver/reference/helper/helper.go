package helper

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

// H implements linearProgram1 from the reference RVO2
// implmentation.
type H struct {
	a  Agent
	cs []plane.HP
}

func New(a Agent) *H {
	return &H{a: a}
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
func (r *H) Add(constraint plane.HP) (vector.V, bool) {
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
		// Any existing optimal solution to the system of constraints
		// must simulaneously satisfy all constraints.
		//
		// Thus, if the entirety of an existing constraint lies on the
		// "invalid" side of the new constraint, then there does not
		// exist a solution which will simultaneously satisfy both
		// constraints. Thus, we need a check for when this occurs and
		// bail out.
		//
		// We can understand this behavior geometrically in the case of
		// two constraints with anti-parallel normals, and with an empty
		// set intersection -- that is, the constraint directions D()
		// are also anti-parallel. The existing solution must lie
		// somewhere in the allowable region of one of these
		// constraints; WLOG, when we add the other constraint, we see
		// that there is no solution which will satisfy both
		// constraints.
		//
		// we define n as the relative orientation between the new
		// constraint and the existing constraint being checked. By our
		// half-plane convention, a relative point p to the "left" of
		// the constraint direction D() is invalid, which can be tested
		// experimentally by
		//
		//   D() x p > 0
		//
		// We construct p to be a vector pointing away from the
		// constraint contributing the D() term; given that there exists
		// two known points that lie on each of the two constraints, we
		// simply take the vector difference between these.
		//
		// N.B.: RVO2 chooses the "right" side of D() to be invalid;
		// thus, their check is instead for n < 0. See
		// https://github.com/snape/RVO2/blob/57098835aa27dda6d00c43fc0800f621724884cc/src/Agent.cpp#L314
		// for an illuminating example: given w is pointing "away" from
		// the VO, the RVO2 implementation chooses to define D() as a
		// clockwise normal, whereas we have chosen to define D() as an
		// anti-clockwise normal to w.
		n := vector.Determinant(constraint.D(), vector.Sub(c.P(), constraint.P()))
		if d < epsilon {
			if n > 0 {
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
