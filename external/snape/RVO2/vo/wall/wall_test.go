package wall

import (
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/external/snape/RVO2/vo/wall/domain"
	"github.com/downflux/go-orca/vo"

	mock "github.com/downflux/go-orca/internal/agent"
)

const (
	delta = 1e-3
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
	r := *segment.New(
		*line.New(
			s.L().L(s.TMax()),
			vector.Scale(-1, s.L().D()),
		),
		s.TMin(),
		s.TMax(),
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
		{
			name: "Left/Circle",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				// agent.V lies within the left end of the
				// scaled obstacle "pill".
				V: vector.Add(s.L().L(s.TMin()), *vector.New(-delta, -delta)),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.LeftCircle,
		},
		{
			// An obstacle pointing in the other direction should
			// not affect the relative orientation domain of the VO
			// object.
			name: "Left/Circle/Flipped",
			vo:   *New(r),
			agent: *mock.New(mock.O{
				V: vector.Add(s.L().L(s.TMin()), *vector.New(-delta, -delta)),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.LeftCircle,
		},
		{
			name: "Right/Circle",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: vector.Add(s.L().L(s.TMax()), *vector.New(delta, -delta)),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.RightCircle,
		},
		{
			name: "Left",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: vector.Add(s.L().L(s.TMin()), *vector.New(-delta, delta)),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.LeftCircle,
		},
		{
			name: "Right",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: vector.Add(s.L().L(s.TMax()), *vector.New(delta, delta)),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.RightCircle,
		},
		{
			name: "Line/Top",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				// V is just above the line in the opposite
				// domain of the agent position.
				V: vector.Add(
					s.L().L(s.L().T(*vector.New(0, 0))),
					*vector.New(0, delta),
				),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.Line,
		},
		{
			name: "Line/Bottom",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				// V is just below the line in the same domain
				// of the agent position.
				V: vector.Add(
					s.L().L(s.L().T(*vector.New(0, 0))),
					*vector.New(0, -delta),
				),
				P: *vector.New(0, -1),
				R: 1,
			}),
			tau:  1,
			want: domain.Line,
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

func TestORCA(t *testing.T) {
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
		want  hyperplane.HP
	}{
		{
			name: "Collision/Line",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: vector.Add(*vector.New(0, 1), *vector.New(0, delta)),
				R: 1,
			}),
			tau: 1,
			want: *hyperplane.New(
				*vector.New(0, 0),
				*vector.New(0, 1),
			),
		},
		{
			name: "Collision/Left",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: vector.Add(*vector.New(-1, 1), *vector.New(-delta, 0)),
				R: 1,
			}),
			tau: 1,
			want: *hyperplane.New(
				*vector.New(0, 0),
				*vector.New(-1, 0),
			),
		},
		{
			name: "Collision/Right",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: *vector.New(0, 0),
				P: vector.Add(*vector.New(1, 1), *vector.New(delta, 0)),
				R: 1,
			}),
			tau: 1,
			want: *hyperplane.New(
				*vector.New(0, 0),
				*vector.New(1, 0),
			),
		},
		{
			name: "Line/Top",
			vo:   *New(s),
			agent: *mock.New(mock.O{
				V: vector.Add(*vector.New(0, 1), *vector.New(0, delta)),
				P: *vector.New(0, 0),
				R: 0.5,
			}),
			tau: 1,
			want: *hyperplane.New(
				*vector.New(-1, 1.5),
				*vector.New(0, 1),
			),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.vo.ORCA(c.agent, c.tau); !hyperplane.Within(got, c.want) {
				t.Errorf("ORCA() = %v, want = %v", got, c.want)
			}
		})
	}
}
