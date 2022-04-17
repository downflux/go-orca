package cache

import (
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/vo/line/cache/domain"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

func TestDomain(t *testing.T) {
	type config struct {
		name string
		c    C
		want domain.D
	}

	f := func(name string, s segment.S, p vector.V, v vector.V, d domain.D) config {
		return config{
			name: name,
			c: *New(
				s,
				/* v = */ *vector.New(0, 0),
				*mock.New(
					mock.O{
						P: p,
						R: 1.0,
						V: v,
					},
				),
				/* tau = */ 1.0,
			),
			want: d,
		}
	}

	testConfigs := append(
		[]config{},
		func() []config {
			// s is a horizontal line segment spanning (-1, 1) to
			// (1, 1).
			s := *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			)
			return []config{
				f(
					"Collision/Left",
					s,
					/* p = */ *vector.New(-2, 1),
					/* v = */ *vector.New(0, 0),
					domain.CollisionLeft,
				),
				f(
					"Collision/Right",
					s,
					*vector.New(2, 1),
					*vector.New(0, 0),
					domain.CollisionRight,
				),
				f(
					"Collision/Top",
					s,
					*vector.New(0, 2),
					*vector.New(0, 0),
					domain.CollisionLine,
				),
				f(
					"Collision/Bottom",
					s,
					*vector.New(0, 0),
					*vector.New(0, 0),
					domain.CollisionLine,
				),
			}
		}()...,
	)

	testConfigs = append(
		testConfigs,
		func() []config {
			// s is a horizontal line segment spanning (-2, 2) to
			// (2, 2).
			s := *segment.New(
				*line.New(
					*vector.New(-2, 2),
					*vector.New(1, 0),
				),
				0,
				4,
			)

			return []config{
				f(
					"Region=3",
					s,
					/* p = */ *vector.New(0, 0),
					/* v = */ *vector.New(0, 4),
					domain.Line,
				),
				f(
					"Region=6",
					s,
					*vector.New(0, 0),
					*vector.New(0, 0),
					domain.Line,
				),
				f(
					"Region=6/Border/1",
					s,
					*vector.New(0, 0),
					*vector.New(-2, 0),
					domain.Line,
				),
				f(
					"Region=6/Border/5",
					s,
					*vector.New(0, 0),
					*vector.New(2, 0),
					domain.Line,
				),
				f(
					"Region=1/Border/2",
					s,
					*vector.New(0, 0),
					*vector.New(-2.5, 4),
					domain.Left,
				),
				f(
					"Region=1/Border/6",
					s,
					*vector.New(0, 0),
					*vector.New(-2.5, 0),
					domain.Left,
				),
				/*
					{
						name: "L/Inside",
					},
					{
						name: "L/Outside",
					},
					{
						name: "R/Inside",
					},
					{
						name: "R/Outside",
					},
				*/
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.c.domain(); got != c.want {
				t.Errorf("domain() = %v, want = %v", got, c.want)
			}
		})
	}
}
