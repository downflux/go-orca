package agent

import (
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

var (
	_ vo.VO = VO{}
)

func TestDomain(t *testing.T) {
	type config struct {
		name string
		vo   VO
		tau  float64
		a    agent.A
		want domain
	}

	testConfigs := append(
		[]config{},
		func() []config {
			vo := *New(
				*segment.New(
					*line.New(
						*vector.New(-1, 1),
						*vector.New(1, 0),
					),
					0,
					2,
				),
				*vector.New(0, 0),
			)
			return []config{
				{
					name: "Collision/Left",
					vo:   vo,
					tau:  1,
					a: mock.New(
						mock.O{
							V: *vector.New(-2, 1),
							R: 1,
							P: *vector.New(0, 0),
						},
					),
					want: collision,
				},
				{
					name: "Collision/Right",
					vo:   vo,
					tau:  1,
					a: mock.New(
						mock.O{
							V: *vector.New(2, 1),
							R: 1,
							P: *vector.New(0, 0),
						},
					),
					want: collision,
				},
				{
					name: "Collision/Top",
					vo:   vo,
					tau:  1,
					a: mock.New(
						mock.O{
							V: *vector.New(-1, 2),
							R: 1,
							P: *vector.New(0, 0),
						},
					),
					want: collision,
				},
				{
					name: "Collision/Bottom",
					vo:   vo,
					tau:  1,
					a: mock.New(
						mock.O{
							V: *vector.New(-1, 0),
							R: 1,
							P: *vector.New(0, 0),
						},
					),
					want: collision,
				},
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.vo.domain(c.a, c.tau); got != c.want {
				t.Errorf("domain() = %v, want = %v", got, c.want)
			}
		})
	}
}
