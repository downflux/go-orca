package segment

import (
	"fmt"
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
	configs := []struct {
		name string

		obstacle segment.S
		p        vector.V
		radius   float64

		l vector.V
		r vector.V
	}{}

	for _, c := range configs {
		s, err := New(c.obstacle, c.p, c.radius)
		if err != nil {
			t.Errorf("New() = _, %v, want = _, nil", err)
		}
		t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
			if got := s.L(); !vector.Within(got, c.l) {
				t.Errorf("L() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := s.R(); !vector.Within(got, c.r) {
				t.Errorf("R() = %v, want = %v", got, c.r)
			}
		})
	}
}
