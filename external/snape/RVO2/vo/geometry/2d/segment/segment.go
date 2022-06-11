package segment

import (
	"fmt"
	"math"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

type S struct {
	// obstacle represents the physical wall in absolute (as opposed to
	// relative) p-space.
	obstacle segment.S

	p      vector.V
	radius float64

	// l is the unit left tangent leg (relative to p), and point towards p.
	// Note that this is anti-parallel to our convention, which has ℓ
	// pointing away from p instead.
	l line.L

	// r is the unit right tangent leg and points away from p.
	r line.L
}

// rotate will rotate the input vector v about the base of v. We define the
// angle theta 𝜃 such that tan(𝜃) = r / l.
func rotate(v vector.V, l float64, r float64, counterclockwise bool) vector.V {
	var u vector.V
	// The counter-clockwise rotation matrix is given by
	//
	// R = [ cos(𝜃) -sin(𝜃) ]
	//     [ sin(𝜃)  cos(𝜃) ]
	//
	// See https://en.wikipedia.org/wiki/Rotation_matrix for more
	// information.
	if counterclockwise {
		u = *vector.New(
			v.X()*l-v.Y()*r,
			v.X()*r+v.Y()*l,
		)
	} else {
		u = *vector.New(
			v.X()*l+v.Y()*r,
			-v.X()*r+v.Y()*l,
		)
	}
	return vector.Scale(1/vector.Magnitude(v), u)
}

func New(obstacle segment.S, p vector.V, radius float64) (*S, error) {
	// rpTMin represents the relative position from the agent to the
	// obstacle.
	rpTMin := vector.Sub(obstacle.L().L(obstacle.TMin()), p)
	rpTMax := vector.Sub(obstacle.L().L(obstacle.TMax()), p)

	// Check for obliqueness. Note that while the distance to the line may
	// be less than the radius, the agent may lie past the ends of the
	// obstacle, meaning the total distance is larger than the radius.
	if d := obstacle.L().Distance(p); d <= radius {
		// The agent lies past the TMin point of the obstacle -- the
		// TMax point of the obstacle is obscured.
		if t := obstacle.L().T(p); t < obstacle.TMin() {
			rpTMax = rpTMin
		} else if t > obstacle.TMax() {
			rpTMin = rpTMax
		}
	}

	// lTMin represents the length of the tangent leg from p the tangent
	// point on the circle around the TMin point of the obstacle.
	var lTMin float64
	var lTMax float64
	if p := vector.Magnitude(rpTMin); p*p-radius*radius <= 0 {
		return nil, fmt.Errorf("cannot construct a walled collision obstacle -- agent is already colliding with the wall")
	} else {
		lTMin = math.Sqrt(p*p - radius*radius)
	}
	if p := vector.Magnitude(rpTMax); p*p-radius*radius <= 0 {
		return nil, fmt.Errorf("cannot construct a walled collision obstacle -- agent is already colliding with the wall")
	} else {
		lTMax = math.Sqrt(p*p - radius*radius)
	}

	// m is rpTMin rotated counter-clockwise about p to lie tangent to the
	// circle around the obstacle TMin point.
	m := rotate(rpTMin, lTMin, radius, true)
	s := rotate(rpTMax, lTMax, radius, false)

	// m is rotated the wrong way -- this means the obstacle is directed in
	// the opposite direction, and actually makes up the "right" leg
	// instead.
	if obstacle.L().T(m) > obstacle.TMin() {
		m = rotate(rpTMax, lTMax, radius, true)
		s = rotate(rpTMin, lTMin, radius, false)
		rpTMin, rpTMax = rpTMax, rpTMin
		lTMin, lTMax = lTMax, lTMin
	}

	// m and s are just rotated position vectors; we need to ensure the
	// tangent vectors have the appropriate length of the sides.
	m = vector.Scale(-lTMin, m)
	s = vector.Scale(lTMax, s)

	// l is the left tangent leg relative to the agent position. We are
	// defining velocity points "outside" of the velocity obstacle as
	// feasible, and by our convention of 2D hyperplanes, the characteristic
	// line of the hyperplane contains infeasible points on the "left" side.
	// That is, l should point from the obstacle to the agent, while WLOG
	// r should point from the agent to the obstacle.
	l := *line.New(
		// Because the left leg is pointing towards p in the official
		// RVO2 implementation, the tangent point is facing the opposite
		// direction of the tangent vector.
		/* p = */
		vector.Add(p, vector.Scale(-1, m)),
		/* d = */ m,
	)
	r := *line.New(p, s)

	obstacle = *segment.New(
		*line.New(rpTMin, vector.Sub(rpTMax, rpTMin)),
		0,
		1,
	)

	return &S{
		obstacle: obstacle,
		radius:   radius,
		p:        p,

		l: l,
		r: r,
	}, nil
}

func (s S) IsLeftNegative() bool { return true }
func (s S) L() line.L            { return s.l }
func (s S) R() line.L            { return s.r }

func (s S) S() segment.S { return s.obstacle }
