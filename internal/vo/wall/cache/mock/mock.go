// TODO(minkezhang): Check for obstacle handedness.
package mock

import (
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"
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

func (vo VO) domain(agent agent.A) domain.D {
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
	dl := vo.obstacle.L().Distance(agent.P())

	t := vo.obstacle.L().T(agent.P())
	if t <= vo.obstacle.TMin() && d1 <= agent.R() {
		return domain.CollisionLeft
	}
	if t >= vo.obstacle.TMax() && d2 <= agent.R() {
		return domain.CollisionRight
	}
	if vo.obstacle.TMin() <= t && t <= vo.obstacle.TMax() && dl <= agent.R() {
		return domain.CollisionLine
	}

	panic("unhandled domain case")
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
	t := vo.obstacle.L().T(agent.P())

	switch d := vo.domain(agent); d {
	case domain.CollisionLeft:
		return *hyperplane.New(
			*vector.New(0, 0),  // VOpt
			vector.Unit(*vector.New(-rp1.X(), -rp1.Y())),
		)
	case domain.CollisionRight:
		return *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(*vector.New(-rp2.X(), -rp2.Y())),
		)
	case domain.CollisionLine:
		return *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(
				vector.Sub(
					agent.P(),
					vo.obstacle.L().L(t),
				),
			),
		)
	}

	panic("unhandled ORCA case")
}
