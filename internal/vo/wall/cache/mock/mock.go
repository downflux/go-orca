// TODO(minkezhang): Check for obstacle handedness.
package mock

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"
)

type VO struct {
	obstacle segment.S
}

func New(obstacle segment.S) *VO {
	return &VO{
		obstacle: *segment.New(
			*line.New(
				obstacle.L().P(),
				vector.Scale(obstacle.TMax()-obstacle.TMin(), obstacle.L().D()),
			),
			0,
			1,
		),
	}
}

func (vo VO) domain(agent agent.A, tau float64) domain.D {
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

	// Check for collisions.
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

	// No collisions -- start considering v-space instead.
	l := *segment.New(
		*line.New(
			vector.Scale(1/tau, vo.obstacle.L().P()),
			vector.Scale(1/tau, vo.obstacle.L().D()),
		),
		vo.obstacle.TMin(),
		vo.obstacle.TMax(),
	)

	t = l.L().T(agent.V())

	// t1 and t2 take into account obliqueness.
	t1 := vo.l(agent).T(agent.V())
	t2 := vo.r(agent).T(agent.V())

	if (t < vo.obstacle.TMin() && t1 < 0) || (vo.oblique(agent) && t1 < 0 && t2 < 0) {
		return domain.Left // LeftCircle
	}
	if t > vo.obstacle.TMax() && t2 < 0 {
		return domain.Right // RightCircle
	}

	dl = l.L().Distance(agent.V())
	d1 = vo.l(agent).Distance(agent.V())
	d2 = vo.r(agent).Distance(agent.V())

	if vo.oblique(agent) || t < vo.obstacle.TMin() || t > vo.obstacle.TMax() {
		dl = math.Inf(1)
	}
	if t1 < 0 {
		d1 = math.Inf(1)
	}
	if t2 < 0 {
		d2 = math.Inf(1)
	}

	if dl <= d1 && dl <= d2 {
		return domain.Line
	}

	if dl <= d2 {
		return domain.Left
	}

	return domain.Right
}

func (vo VO) oblique(agent agent.A) bool {
	d := vo.obstacle.L().Distance(agent.P())
	t := vo.obstacle.L().T(agent.P())
	return d < agent.R() && (t < vo.obstacle.TMin() || t > vo.obstacle.TMax())
}

func (vo VO) l(agent agent.A) line.L {
	d := vo.obstacle.L().Distance(agent.P())
	t := vo.obstacle.L().T(agent.P())
	oblique_right := d < agent.R() && t > vo.obstacle.TMax()

	o := vo.obstacle.L().L(vo.obstacle.TMin())
	if oblique_right {
		o = vo.obstacle.L().L(vo.obstacle.TMax())
	}

	rp := vector.Sub(o, agent.P())
	l := math.Sqrt(vector.SquaredMagnitude(rp) - agent.R()*agent.R())

	return *line.New(
		/* p = */ o,
		/* d = */ vector.Scale(
			1/vector.SquaredMagnitude(rp),
			*vector.New(
				rp.X()*l-rp.Y()*agent.R(),
				rp.X()*agent.R()+rp.Y()*l,
			),
		),
	)

}

func (vo VO) r(agent agent.A) line.L {
	d := vo.obstacle.L().Distance(agent.P())
	t := vo.obstacle.L().T(agent.P())
	oblique_left := d < agent.R() && t < vo.obstacle.TMin()

	o := vo.obstacle.L().L(vo.obstacle.TMax())
	if oblique_left {
		o = vo.obstacle.L().L(vo.obstacle.TMin())
	}
	rp := vector.Sub(o, agent.P())
	r := math.Sqrt(vector.SquaredMagnitude(rp) - agent.R()*agent.R())

	return *line.New(
		o,
		vector.Scale(
			1/vector.SquaredMagnitude(rp),
			*vector.New(
				rp.X()*r+rp.Y()*agent.R(),
				-rp.X()*agent.R()+rp.Y()*r,
			),
		),
	)
}

func (vo VO) ORCA(agent agent.A, tau float64) hyperplane.HP {
	rp1 := vector.Sub(
		vo.obstacle.L().L(vo.obstacle.TMin()),
		agent.P(),
	)
	rp2 := vector.Sub(
		vo.obstacle.L().L(vo.obstacle.TMax()),
		agent.P(),
	)
	t := vo.obstacle.L().T(agent.P())

	d := vo.domain(agent, tau)
	switch d {
	case domain.CollisionLeft:
		return *hyperplane.New(
			*vector.New(0, 0), // VOpt
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

	// No collisions -- start considering v-space instead.
	l := *segment.New(
		*line.New(
			vector.Scale(1/tau, vo.obstacle.L().P()),
			vector.Scale(1/tau, vo.obstacle.L().D()),
		),
		vo.obstacle.TMin(),
		vo.obstacle.TMax(),
	)

	t = l.L().T(agent.V())

	// t1 and t2 take into account obliqueness.
	t1 := vo.l(agent).T(agent.V())
	t2 := vo.r(agent).T(agent.V())

	switch d {
	case domain.Left:
		// LeftCircle
		//
		// TODO(minkezhang): Figure out if TMin is the correct value
		// here.
		if (t < vo.obstacle.TMin() && t1 < 0) || (vo.oblique(agent) && t1 < 0 && t2 < 0) {
			w := vector.Unit(
				vector.Sub(vo.l(agent).P(), agent.V()))
			return *hyperplane.New(
				vector.Add(
					vo.l(agent).P(),
					vector.Scale(agent.R() / tau, w),
				),
				w,
			)
		}
	case domain.Right:
		// RightCircle
		if t > vo.obstacle.TMax() && t2 < 0 {
			w := vector.Unit(
				vector.Sub(vo.r(agent).P(), agent.V()))
			return *hyperlane.New(
				vector.Add(
					vo.r(agent).P(),
					vector.Scale(agent.R() / tau, w),
				),
				w,
			)
		}
	}

	dl = l.L().Distance(agent.V())
	d1 = vo.l(agent).Distance(agent.V())
	d2 = vo.r(agent).Distance(agent.V())

	if vo.oblique(agent) || t < vo.obstacle.TMin() || t > vo.obstacle.TMax() {
		dl = math.Inf(1)
	}
	if t1 < 0 {
		d1 = math.Inf(1)
	}
	if t2 < 0 {
		d2 = math.Inf(1)
	}

	switch d {
	case domain.Left:
		return *hyperplane.New(
			vector.Add(
				l.L().L(l.M
	case domain.Right:
	case domain.Line:
	}
	if dl <= d1 && dl <= d2 {
		return domain.Line
	}

	if dl <= d2 {
		return domain.Left
	}

	return domain.Right

	panic("unhandled ORCA case")
}
