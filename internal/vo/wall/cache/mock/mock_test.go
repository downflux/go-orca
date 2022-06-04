package mock

import (
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"

	mock "github.com/downflux/go-orca/internal/agent"
)

var (
	_ vo.VO = VO{}
)

func TestDomain(t *testing.T) {
	s := *segment.New(
		*line.New(
			*vector.New(-1, 1),
			*vector.New(1, 0),
		),
		0,
		2,
	)

	testConfigs := []struct {
		name  string
		vo    VO
		agent agent.A
		tau   float64
		want  domain.D
	}{
		{
			name: "Collision/Line",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: *vector.New(0, 1),
				R: 1,
			}),
			tau:  1,
			want: domain.CollisionLine,
		},
		{
			name: "Collision/Left",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: *vector.New(-1.5, 1),
				R: 1,
			}),
			tau:  1,
			want: domain.CollisionLeft,
		},
		{
			name: "Collision/Right",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: *vector.New(1.5, 1),
				R: 1,
			}),
			tau:  1,
			want: domain.CollisionRight,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.vo.domain(c.agent, c.tau); got != c.want {
				t.Errorf("domain() = %v, want = %v", got, c.want)
			}
		})
	}
}
