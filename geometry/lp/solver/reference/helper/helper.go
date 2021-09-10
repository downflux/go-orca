package helper

import (
	"math"

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

const (
	epsilon = 1e-10
)

// Agent represents the circular constraint and the optimization target.
//
// TODO(minkezhang): Rename to be less suggestive of this always representing a
// physical moving object.
type Agent interface {
	S() float64
	T() vector.V
}

// H implements linearProgram1 from the reference RVO2
// implmentation.
type H struct {
	a  Agent
	cs []plane.HP

	// knownOptimalMagnitude indicates that we should only optimize the
	// direction of the result -- this is passed down from LinearProgram3 in
	// the reference RVO2 implementation. LP3 is called when there is no
	// feasible solution to the 2D constraints; in this case, we want to
	// move the object at max speed, with a velocity directed "into" the
	// valid v-space region.
	knownOptimalMagnitude bool
}

func New(a Agent, knownOptimalMagnitude bool) *H {
	return &H{a: a, knownOptimalMagnitude: knownOptimalMagnitude}
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
	//
	// See https://stackoverflow.com/a/1084899 for more information.
	tl := -dot - math.Sqrt(discriminant)
	tr := -dot + math.Sqrt(discriminant)

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
		//
		// Given two constraints L, M, we need to find their
		// intersection; WLOG, let's project the intersection point onto
		// L.
		//
		// We know the parametric equation form of these lines -- that
		// is,
		//
		//   L = P + tD
		//   M = Q + uE
		//
		// At their intersection, we know that L meets M:
		//
		//   L = M
		//   => P + tD = Q + uE
		//
		// We want to find the projection onto L, which means we need to
		// find a concrete value for t. the other parameter u doesn't
		// matter so much -- let's try to get rid of it.
		//
		//   uE = P - Q + tD
		//
		// Here, we know P, D, Q, and E are vectors, and we can
		// decompose these into a system of equations by isolating their
		// orthogonal (e.g. horizontal and vertical) components.
		//
		//   uEx = Px - Qx + tDx
		//   uEy = Py - Qy + tDy
		//
		// Solving for u, we get
		//
		//   (Px - Qx + tDx) / Ex = (Py - Qy + tDy) / Ey
		//   => Ey (Px - Qx + tDx) = Ex (Py - Qy + tDy)
		//
		// We leave the task of simplifying the above terms as an
		// exercise to the reader. Isolating t, and noting some common
		// substitutions, we get
		//
		//   t = || E x (P - Q) || / || D x E ||
		//
		// See https://gamedev.stackexchange.com/a/44733 for more
		// information.
		t := n / d
		if d > 0 {
			// We are "shrinking" the possible range of t-values for
			// L that the solution may project onto. The absolute
			// min and max t-values are initially set as the
			// max-speed circle intersection points; as we add more
			// constraints, we progressively refine our valid
			// interval by the constraint-constraint intersection
			// points.
			tr = math.Min(tr, t)
		} else {
			tl = math.Max(tl, t)
		}

		if tl > tr {
			return vector.V{}, false
		}
	}

	var tOpt float64

	// We are choosing the maximal valid t-value on constraints where the
	// target vector lies in the valid region, and choosing the minimal
	// t-value otherwise.
	//
	// Note that tl and tr are orientation-invariant, i.e. if we flip the
	// constraint direction, tl and tr are still the same values; thus, we
	// are choosing a distinct root for each contraint orientation.
	if r.knownOptimalMagnitude {
		if vector.Dot(constraint.D(), r.a.T()) < 0 {
			tOpt = tl
		} else {
			tOpt = tr
		}
	} else {
		// We represent the linear constraint as a vector
		//
		//   L := P + tD
		//
		// We want to find the point on L which is closest to the agent goal
		// velocity G; this is equivalent to finding the distance between G and
		// L, and solving for t_min.
		tOpt = vector.Dot(constraint.D(), vector.Sub(r.a.T(), constraint.P()))

		// If t_min lies beyond tl or tr, we know that t_min will fail to
		// satisfy at least one constraint (i.e. lies outside the boundaries of
		// at least one half-plane). The "best" we can do is to bound our
		// solution to the parametric bounds.
		//
		// If t_min lies between tl and tr, then we know "optimal" t value is
		// t_min and we can substitute directly into the result.
		if tOpt < tl {
			tOpt = tl
		} else if tOpt > tr {
			tOpt = tr
		}
	}

	r.cs = append(r.cs, constraint)
	return vector.Add(constraint.P(), vector.Scale(tOpt, constraint.D())), true
}
