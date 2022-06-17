package segment

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"

	mock "github.com/downflux/go-orca/external/snape/RVO2/vo/geometry/2d/segment"
	ov "github.com/downflux/go-orca/internal/geometry/2d/vector"
)

type shim mock.S

func (s shim) L() line.L {
	l := mock.S(s).L()
	return *line.New(
		/* p = */ vector.Add(l.P(), l.D()),
		/* d = */ vector.Scale(-1, l.D()),
	)
}
func (s shim) R() line.L {
	r := mock.S(s).R()
	return *line.New(
		vector.Add(r.P(), r.D()),
		vector.Scale(-1, r.D()),
	)
}

func rn() float64   { return rand.Float64()*200 - 100 }
func rv() vector.V  { return *vector.New(rn(), rn()) }
func rs() segment.S { return *segment.New(*line.New(rv(), rv()), rn(), rn()) }

// TestOrientation verifies L() and R() orientations follow the existing
// convention -- that is, L() points from the base to the tangent point on the
// circle, and R() is directed towards the base.
func TestOrientation(t *testing.T) {

}

func TestConformance(t *testing.T) {
	n := 1000
	type config struct {
		name string
		s    segment.S
		r    float64
		p    vector.V
	}
	testConfigs := []config{
		config{
			name: "Trivial",
			s: *segment.New(
				*line.New(
					/* p = */ *vector.New(-2, 2),
					/* d = */ *vector.New(1, 0),
				),
				0,
				4,
			),
			p: *vector.New(1, 1),
			r: 0.5,
		},
	}
	for i := 0; i < n; i++ {
		c := &config{
			name: fmt.Sprintf("Random-%v", i),
			s:    rs(),
			p:    rv(),
		}
		c.r = rand.Float64() * c.s.L().Distance(c.p)
		testConfigs = append(testConfigs, *c)
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			s := New(c.s, c.p, c.r)
			r, err := mock.New(c.s, c.p, c.r)
			if err != nil {
				t.Errorf("New() = _, %v, want = _, nil", err)
			}

			t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
				got := s.L()
				want := shim(*r).L()
				if !line.WithinEpsilon(got, want, epsilon.Absolute(1e-5)) {
					t.Errorf("L() = %v, want = %v", got, want)
				}
			})
			t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
				got := s.R()
				want := shim(*r).R()
				if !line.WithinEpsilon(got, want, epsilon.Absolute(1e-5)) {
					t.Errorf("R() = %v, want = %v", got, want)
				}
			})
		})
	}
}

func TestL(t *testing.T) {
	type config struct {
		name string
		s    S
		l    vector.V
		r    vector.V
	}

	testConfigs := []config{
		// s is a horizontal line segment spanning (-1, 1) to
		// (1, 1).
		{
			name: "Normal",
			s: *New(
				*segment.New(
					*line.New(
						*vector.New(-1, 1),
						*vector.New(1, 0),
					),
					0,
					2,
				),
				*vector.New(0, 0),
				1.0,
			),
			l: *vector.New(-1, 0),
			r: *vector.New(-1, 0),
		},
		{
			name: "Normal/Mirror",
			s: *New(
				*segment.New(
					*line.New(
						*vector.New(-1, -1),
						*vector.New(1, 0),
					),
					0,
					2,
				),
				*vector.New(0, 0),
				1.0,
			),
			l: *vector.New(1, 0),
			r: *vector.New(1, 0),
		},
		{
			name: "Oblique/Left",
			s: *New(
				*segment.New(
					*line.New(
						*vector.New(2, 0),
						*vector.New(1, 0),
					),
					0,
					2,
				),
				*vector.New(0, 0),
				1.0,
			),
			l: *vector.New(1.5, math.Sqrt(3)/2),
			r: *vector.New(-1.5, math.Sqrt(3)/2),
		},
		{
			name: "Oblique/Right",
			s: *New(
				*segment.New(
					*line.New(
						*vector.New(-4, 0),
						*vector.New(1, 0),
					),
					0,
					2,
				),
				*vector.New(0, 0),
				1.0,
			),
			l: *vector.New(-1.5, -math.Sqrt(3)/2),
			r: *vector.New(1.5, -math.Sqrt(3)/2),
		},
	}

	for _, c := range testConfigs {
		t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
			if got := c.s.L().D(); !vector.WithinEpsilon(c.l, got, epsilon.Absolute(1e-5)) {
				t.Errorf("L().D() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := c.s.R().D(); !vector.WithinEpsilon(c.r, got, epsilon.Absolute(1e-5)) {
				t.Errorf("R().D() = %v, want = %v", got, c.r)
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/SR", c.name), func(t *testing.T) {
			// Also cover the oblique case where the segment does
			// not have a well-defined direction.
			if !ov.IsNormalOrientation(c.s.S().L().D(), c.s.R().D()) && !vector.Within(c.s.S().L().D(), *vector.New(0, 0)) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/LS", c.name), func(t *testing.T) {
			// Also cover the oblique case where the segment does
			// not have a well-defined direction.
			if !ov.IsNormalOrientation(c.s.L().D(), c.s.S().L().D()) && !vector.Within(c.s.S().L().D(), *vector.New(0, 0)) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/LR", c.name), func(t *testing.T) {
			if !ov.IsNormalOrientation(c.s.L().D(), c.s.R().D()) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
	}
}
