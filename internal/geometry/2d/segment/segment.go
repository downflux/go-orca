// Package segment defines a truncated cone-like object whose bottom is defined
// by a characteristic line segment (instead of a point). This object also has a
// characteristic "turning radius", which defines the sharpness of the curve
// from the bottom line segment to the edges.
//
//   L \     / R
//      \___/
//        S
//
// As with the case of the point-defined cone, we define tangential lines from
// the VO to the left and right circles of at the ends of the line segment S.
package segment

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/geometry/2d/cone"
)

type S struct {
	// s represents the relative physical line segment of the obstacle.
	s segment.S

	p vector.V

	// r is the thickness of the line segment.
	radius float64

	l line.L
	r line.L
}

func New(s segment.S, p vector.V, radius float64) *S {
	rpTMin := vector.Sub(s.L().L(s.TMin()), p)
	rpTMax := vector.Sub(s.L().L(s.TMax()), p)

	// Check for obliqueness.
	if d := s.L().Distance(p); d < radius {
		// If the agent lies past the TMin point of the obstacle (and
		// the distance to the projected line defind by the obstacle),
		// then the far end of the obstacle is obscured.
		if t := s.L().T(p); t < s.TMin() {
			rpTMax = rpTMin
		} else if t > s.TMax() {
			rpTMin = rpTMax
		}
	}

	cTMin, err := cone.New(*hypersphere.New(rpTMin, radius))
	if err != nil {
		panic(
			fmt.Sprintf(
				"could not construct line segment VO object: %v",
				err))
	}
	cTMax, err := cone.New(*hypersphere.New(rpTMax, radius))
	if err != nil {
		panic(
			fmt.Sprintf(
				"could not construct line segment VO object: %v",
				err))
	}

	// If the end of the left tangent leg lies past the left segment,
	// we flip the definition of the left and right orientations.
	if s.L().T(vector.Add(p, cTMin.L().D())) > s.TMin() {
		cTMin, cTMax = cTMax, cTMin
		rpTMin, rpTMax = rpTMax, rpTMin
	}
	l := *line.New(
		vector.Add(p, cTMin.L().P()),
		cTMin.L().D(),
	)
	r := *line.New(
		vector.Add(p, cTMax.R().P()),
		cTMax.R().D(),
	)

	// Note that the segment flows from the right to the left. This
	// preserves the normal orientation between L, R, and S.
	s = *segment.New(
		*line.New(rpTMax, vector.Sub(rpTMin, rpTMax)),
		0,
		1,
	)

	return &S{
		s:      s,
		p:      p,
		radius: radius,

		l: l,
		r: r,
	}
}

// S returns the base of line segment VO. Depending on the setup, this segment
// may differ from the constructor input segment.
func (s S) S() segment.S { return s.s }

// L calculates the left vector of the tangent line from the agent position to
// the base of the truncated line segment.
//
// N.B.: ℓ is always directed away from the agent.
func (s S) L() line.L { return s.l }
func (s S) R() line.L { return s.r }
