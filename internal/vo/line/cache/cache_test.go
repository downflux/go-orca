package cache

import (
	"fmt"
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/vo/line/cache/domain"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

func TestL(t *testing.T) {
	type config struct {
		name string
		c    C
		l    vector.V
		r    vector.V
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
			tau := 1.0
			v := *vector.New(0, 0)
			r := 1.0

			return []config{
				{
					name: "Normal",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								P: *vector.New(0, 0),
								R: r,
							},
						),
						tau,
					),
					// L points away from the agent.
					l: *vector.New(-1, 0),
					// R points toward the agent.
					r: *vector.New(-1, 0),
				},
				{
					name: "Normal/Mirror",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								P: *vector.New(0, 2),
								R: r,
							},
						),
						tau,
					),
					l: *vector.New(1, 0),
					r: *vector.New(1, 0),
				},
				// TODO(minkezhang): Add oblique cases.
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
			if got := c.c.l(); !vector.Within(c.l, got) {
				t.Errorf("l() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := c.c.r(); !vector.Within(c.r, got) {
				t.Errorf("r() = %v, want = %v", got, c.r)
			}
		})
	}
}

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
			tau := 1.0
			v := *vector.New(0, 0)
			r := 1.0
			return []config{
				{
					name: "Collision/Left",
					c: *New(
						s,
						v,
						*mock.New(
							mock.O{
								P: *vector.New(-2, 1),
								R: r,
							},
						),
						tau,
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
								P: *vector.New(2, 1),
								R: r,
							},
						),
						tau,
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
								P: *vector.New(0, 2),
								R: r,
							},
						),
						tau,
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
								P: *vector.New(0, 0),
								R: r,
							},
						),
						tau,
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

func TestW(t *testing.T) {
	type config struct {
		name string
		c    C
		want vector.V
	}

	testConfigs := []config{}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.c.w(); !vector.Within(got, c.want) {
				t.Errorf("w() = %v, want = %v", got, c.want)
			}
		})
	}
}
