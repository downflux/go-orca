package cache

import (
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/vo/line/domain"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

func TestDomain(t *testing.T) {
	type config struct {
		name string
		c    C
		want domain.D
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
			v := *vector.New(0, 0)
			return []config{
				{
					name: "Collision/Left",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								V: *vector.New(-2, 1),
								R: 1,
								P: *vector.New(0, 0),
							},
						),
						1,
					),
					want: domain.CollisionLeft,
				},
				{
					name: "Collision/Right",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								V: *vector.New(2, 1),
								R: 1,
								P: *vector.New(0, 0),
							},
						),
						1,
					),
					want: domain.CollisionRight,
				},
				{
					name: "Collision/Top",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								V: *vector.New(0, 2),
								R: 1,
								P: *vector.New(0, 0),
							},
						),
						1,
					),
					want: domain.CollisionLine,
				},
				{
					name: "Collision/Bottom",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								V: *vector.New(0, 0),
								R: 1,
								P: *vector.New(0, 0),
							},
						),
						1,
					),
					want: domain.CollisionLine,
				},
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
