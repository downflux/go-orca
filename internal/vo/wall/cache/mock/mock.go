// TODO(minkezhang): Check for obstacle handedness.
package mock

import (
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
)

type VO struct {
	obstacle segment.S
}

func New(obstacle segment.S) *VO {
	return &VO{
		obstacle: *segment.New(
			*line.New(
				obstacle.L().P(),
				vector.Scale(obstacle.TMax() - obstacle.TMin(), obstacle.L().D()),
			),
			0,
			1,
		),
	}
}

func (vo VO) ORCA(agent agent.A) hyperplane.HP {
	rp1 := vector.Sub(
		vo.obstacle.L().L(vo.obstacle.TMin()),
		agent.P(),
	)
	rp2 := vector.Sub(
		vo.obstacle.L().L(vo.obstacle.TMax()),
		agent.P(),
	)
	d1 := vector.Magnitude(rp1)
	d2 := vector.Magnitude(rp2)

	t := vo.obstacle.T(rp1)
	d := vector.Magnitude(vector.Sub(agent.P(), vo.obstacle.L().L(t)))

	// Agent collides with the left vertex
	if t < 0 && d1 <= agent.R() {
		return *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(*vector.New(-rp1.Y(), rp1.X())),
		)
	// Agent collides with the right vertex
	} else if t > 1 && d2 <= agent.R() {
		return *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(*vector.New(-rp2.Y(), rp2.X())),
		)
	// Agent collides with the obstacle
	} else if t >= 0 && t <= 1 && d < agent.R() {
		return *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(vector.Sub(agent.P(), vo.obstacle.L().L(t))),
		)
	}

	l1 :=
	panic("unhandled ORCA case")
}
