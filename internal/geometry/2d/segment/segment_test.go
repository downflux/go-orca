package segment

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-orca/external/snape/RVO2/vo/geometry/2d/segment"
	ov "github.com/downflux/go-orca/internal/geometry/2d/vector"
)

func rn() float64   { return rand.Float64()*200 - 100 }
func rv() vector.V  { return *vector.New(rn(), rn()) }
func rs() segment.S { return *segment.New(*line.New(rv(), rv()), rn(), rn()) }

func TestConformance(t *testing.T) {
	n := 1000
	type config struct {
		name string
		s    segment.S
		r    float64
		p    vector.V
	}
	var testConfigs []config
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
				want := r.L()
				if !vector.Within(got, want) {
					t.Errorf("L() = %v, want = %v", got, want)
				}
			})
			t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
				got := s.R()
				want := r.R()
				if !vector.Within(got, want) {
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
			if got := c.s.L(); !vector.Within(c.l, got) {
				t.Errorf("L() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := c.s.R(); !vector.Within(c.r, got) {
				t.Errorf("R() = %v, want = %v", got, c.r)
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/LS", c.name), func(t *testing.T) {
			if !ov.IsNormalOrientation(c.s.L(), c.s.S().L().D()) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/SR", c.name), func(t *testing.T) {
			if !ov.IsNormalOrientation(c.s.S().L().D(), c.s.R()) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
		t.Run(fmt.Sprintf("%v/Orientation/LR", c.name), func(t *testing.T) {
			if !ov.IsNormalOrientation(c.s.L(), c.s.R()) {
				t.Errorf("IsNormalOrientation() == false, want = true")
			}
		})
	}
}
