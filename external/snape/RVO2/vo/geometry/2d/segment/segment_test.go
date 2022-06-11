package segment

import (
	"fmt"
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestCollision(t *testing.T) {
	configs := []struct {
		name string

		obstacle segment.S
		p        vector.V
		radius   float64

		success bool
	}{
		{
			name: "Collision/Left",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(-1.5, 1),
			radius: 1,

			success: false,
		},
		{
			name: "Collision/Right",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(1.5, 1),
			radius: 1,

			success: false,
		},
		{
			name: "Collision/Line",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(0, 1),
			radius: 1,

			success: false,
		},
		{
			name: "Success/Left",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(-100, 1),
			radius: 1,

			success: true,
		},
		{
			name: "Success/Right",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(100, 1),
			radius: 1,

			success: true,
		},
		{
			name: "Success/Line",
			obstacle: *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			),
			p:      *vector.New(0, 100),
			radius: 1,

			success: true,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if _, err := New(c.obstacle, c.p, c.radius); c.success && err != nil {
				t.Errorf("New() = _, %v, want = _, nil", err)
			} else if !c.success && err == nil {
				t.Errorf("New() = _, nil, want a non-nil error")
			}
		})
	}
}

func TestTangents(t *testing.T) {
	o := *segment.New(
		*line.New(
			*vector.New(-1, 1),
			*vector.New(1, 0),
		),
		0,
		2,
	)
	type config struct {
		name string

		obstacle segment.S
		p        vector.V
		radius   float64

		l vector.V
		r vector.V
	}

	var testConfigs []config
	testConfigs = append(testConfigs, func() []config {
		// l is always left negative, i.e. directed from the
		// obstacle to agent.P.
		l := vector.Scale(-1/2.0, *vector.New(
			-math.Sqrt(1.75)-0.5,
			-0.5+math.Sqrt(1.75)),
		)
		r := vector.Scale(1/2.0, *vector.New(
			math.Sqrt(1.75)+0.5,
			-0.5+math.Sqrt(1.75)),
		)

		return []config{
			{
				name:     "Trivial",
				obstacle: o,
				p:        *vector.New(0, 0),
				radius:   0.5,
				l:        l,
				r:        r,
			},
			{
				// Check that even if the obstacle segment
				// definition is flipped, the left and right
				// tangent vectors are invariant.
				name: "Trivial/Flipped",
				obstacle: *segment.New(
					*line.New(
						o.L().L(o.TMax()),
						vector.Scale(-1, o.L().D()),
					),
					o.TMin(),
					o.TMax(),
				),
				p:      *vector.New(0, 0),
				radius: 0.5,

				l: l,
				r: r,
			},
		}
	}()...)

	testConfigs = append(testConfigs, config{
		name:     "Trivial/Mirrored",
		obstacle: o,
		p:        *vector.New(0, 2),
		radius:   0.5,

		l: vector.Scale(1/2.0, *vector.New(
			-math.Sqrt(1.75)-0.5,
			-0.5+math.Sqrt(1.75)),
		),
		r: vector.Scale(-1/2.0, *vector.New(
			math.Sqrt(1.75)+0.5,
			-0.5+math.Sqrt(1.75)),
		),
	}, config{
		name:     "Oblique/Left",
		obstacle: o,
		p:        *vector.New(-2, 1),
		radius:   0.5,

		l: vector.Scale(-1, *vector.New(math.Sqrt(0.75), 0.5)),
		r: *vector.New(math.Sqrt(0.75), -0.5),
	}, config{
		name:     "Oblique/Right",
		obstacle: o,
		p:        *vector.New(2, 1),
		radius:   0.5,

		l: vector.Scale(-1, *vector.New(-math.Sqrt(0.75), -0.5)),
		r: *vector.New(-math.Sqrt(0.75), 0.5),
	})

	for _, c := range testConfigs {
		s, err := New(c.obstacle, c.p, c.radius)
		if err != nil {
			t.Errorf("New() = _, %v, want = _, nil", err)
		}
		t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
			if got := vector.Unit(s.L().D()); !vector.Within(got, c.l) {
				t.Errorf("L() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := vector.Unit(s.R().D()); !vector.Within(got, c.r) {
				t.Errorf("R() = %v, want = %v", got, c.r)
			}
		})
	}
}
