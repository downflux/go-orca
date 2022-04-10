package segment

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/geometry/cone"

	ov "github.com/downflux/go-orca/internal/geometry/2d/vector"
)

type S struct {
	// s represents the relative physical line segment of the obstacle.
	s segment.S

	// r is the thickness of the line segment.
	r float64

	cl cone.C
	cr cone.C
}

func New(s segment.S, r float64) *S {
	cTMin, err := cone.New(*hypersphere.New(s.L().L(s.TMin()), r))
	if err != nil {
		panic(
			fmt.Sprintf(
				"could not construct line segment VO object: %v",
				err))
	}
	cTMax, err := cone.New(*hypersphere.New(s.L().L(s.TMax()), r))
	if err != nil {
		panic(
			fmt.Sprintf(
				"could not construct line segment VO object: %v",
				err))
	}

	cl := cTMin
	cr := cTMax

	t := s.L().T(*vector.New(0, 0))
	d := s.L().Distance(*vector.New(0, 0))

	if (
	// The right truncation circle is obstructing the view of the
	// left end of the line segment. Use the right circle to
	// calculate the left tangent leg.
	t >= s.TMax() && d <= r) || (
	// The agent is flipped across the truncation line.
	vector.Determinant(
		s.L().D(),
		s.L().L(s.TMin()),
	) < 0) {
		cl = cTMax
	}
	if (
	// The left truncation circle is obstructing the view of the
	// right end of the line segment. Use the left circle to
	// calculate the right tangent leg.
	t <= s.TMin() && d <= r) || (
	// The agent is flipped across the truncation line.
	vector.Determinant(
		s.L().D(),
		s.L().L(s.TMin()),
	) < 0) {
		cr = cTMin
	}

	return &S{
		s:  s,
		r:  r,
		cl: *cl,
		cr: *cr,
	}
}

func (s S) S() segment.S {
	v := s.s
	// In the oblique case, we need to generate a new segment which
	// preserves the relative orientation between L, S, and R. We take as
	// the segment direction L + R, and set the root of the segment at the
	// base of the cone(s).
	if hypersphere.Within(s.cl.C(), s.cr.C()) {
		v = *segment.New(
			*line.New(
				s.cl.C().P(),
				vector.Add(
					s.L(),
					s.R(),
				),
			),
			0,
			0,
		)
	}
	if !ov.IsNormalOrientation(s.L(), v.L().D()) {
		v = *segment.New(
			*line.New(
				s.s.L().L(s.s.TMax()),
				vector.Scale(-1, v.L().D()),
			),
			v.TMin(),
			v.TMax(),
		)
	}
	return v
}
func (s S) L() vector.V { return s.cl.L() }
func (s S) R() vector.V { return s.cr.R() }
