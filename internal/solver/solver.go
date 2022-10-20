package solver

import (
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/geometry/2d/constraint"
	"github.com/downflux/go-orca/internal/solver/bounds/circular"
	"github.com/downflux/go-orca/internal/solver/feasibility"

	s2d "github.com/downflux/go-orca/internal/solver/2d"
	s3d "github.com/downflux/go-orca/internal/solver/3d"
)

// project finds the point on the input line segment which is closest to the
// input vector.
//
// Note that s.T(v) finds the projected parametric t-value of the underlying
// line, bounded by the min / max values of the line segment.
//
// Adding a noise function here to the output will randomize the path of the
// agent a bit, e.g.
//
//	weight := 1e-3
//	dt := rand.Float64() * (s.TMax() - s.TMin()) * weight
//	v := s.L().L(s.T(v) + dt)
//
// This will help with the case where two slow-moving agents become stuck while
// trying to move directly past each other (i.e. agent targets are directly
// behind the opposing agent).
//
// This may be feasible, but other solutions to the problem exist, e.g. in a
// tile-based map, set the actual target to some randomized value around the
// input target, and re-adjust the target once an agent is inside the correct
// tile.
func project(s segment.S, v vector.V) vector.V {
	return s.L().L(s.T(v))
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v, with maximum length of v set to
// r.
func Solve(cs []constraint.C, v vector.V, r float64) vector.V {
	m := *circular.New(r)
	// Ensure the desired target velocity is within the initial bounding
	// constraints.
	if !m.In(v) {
		v = m.V(v)
	}

	u, f := s2d.Solve(m, cs, func(s segment.S) vector.V {
		return project(s, v)
	}, v)

	if f == feasibility.Partial {
		u, f = s3d.Solve(m, cs, u)
	}
	if f != feasibility.Feasible {
		panic("cannot solve linear programming problem for the given set of ORCA lines")
	}

	return u
}
