package wall

import (
	"fmt"
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/external/snape/RVO2/vo/wall/domain"

	vosegment "github.com/downflux/go-orca/external/snape/RVO2/vo/geometry/2d/segment"
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
	domain, _ := vo.orca(agent, tau)
	return domain
}

func (vo VO) orca(agent agent.A, tau float64) (domain.D, hyperplane.HP) {
	// Check for collisions.
	if domain, hp, ok := func() (domain.D, hyperplane.HP, bool) {
		pTMin := vector.Sub(
			vo.obstacle.L().L(vo.obstacle.TMin()),
			agent.P(),
		)
		pTMax := vector.Sub(
			vo.obstacle.L().L(vo.obstacle.TMax()),
			agent.P(),
		)

		dTMin := vector.Magnitude(pTMin)
		dTMax := vector.Magnitude(pTMax)
		d := vo.obstacle.L().Distance(agent.P())

		t := vo.obstacle.L().T(agent.P())
		if t <= vo.obstacle.TMin() && dTMin <= agent.R() {
			return domain.CollisionLeft, *hyperplane.New(
				/* p = */ *vector.New(0, 0), // VOpt
				/* n = */ vector.Unit(vector.Scale(-1, pTMin)),
			), true

		}
		if t >= vo.obstacle.TMax() && dTMax <= agent.R() {
			return domain.CollisionRight, *hyperplane.New(
				*vector.New(0, 0),
				vector.Unit(vector.Scale(-1, pTMax)),
			), true
		}
		if vo.obstacle.TMin() <= t && t <= vo.obstacle.TMax() && d <= agent.R() {
			return domain.CollisionLine, *hyperplane.New(
				*vector.New(0, 0),
				// TODO(minkezhang): Handle the NaN case when
				// the agent lies on the line.
				vector.Unit(
					vector.Sub(
						agent.P(),
						vo.obstacle.L().L(t),
					),
				),
			), true
		}
		return domain.D(0), hyperplane.HP{}, false
	}(); ok {
		return domain, hp
	}

	s, err := vosegment.New(
		// calculate the truncated base of the VO cone in v-space.
		/* s = */
		*segment.New(
			*line.New(
				vector.Scale(1/tau, vector.Sub(
					vo.obstacle.L().P(),
					agent.P())),
				vector.Scale(1/tau, vo.obstacle.L().D())),
			vo.obstacle.TMin(),
			vo.obstacle.TMax(),
		),
		/* p = */ *vector.New(0, 0),
		/* r = */ agent.R()/tau,
	)
	if err != nil {
		panic(fmt.Sprintf("error while constructing the truncated VO cone base: %v", err))
	}

	// RVO2 constructs the left and right tangent legs to both be directed
	// away from the origin -- we are matching that construction here.
	l := *line.New(
		s.S().L().L(s.S().TMin()),
		vector.Scale(-1, s.L().D()))
	r := *line.New(
		s.S().L().L(s.S().TMax()),
		s.R().D())

	// TODO(minkezhang): Test oblique case.
	oblique := vector.Within(l.P(), r.P())

	// t is the projected parametric value along the truncated base. Note
	// that the segment reported by s.S() is normalized, so t is always in
	// [0, 1]. In the oblique case, the line segment is not well-defined, so
	// we use a placeholder value here.
	t := map[bool]float64{
		true:  0.5,
		false: s.S().L().T(agent.V()),
	}[oblique]
	tl := l.T(agent.V())
	tr := r.T(agent.V())

	if (t < 0 && tl < 0) || (oblique && tl < 0 && tr < 0) {
		w := vector.Sub(agent.V(), l.P())
		return domain.LeftCircle, *hyperplane.New(
			// Hyperplane lies tangent to the VO object.
			/* p = */
			line.New(l.P(), vector.Unit(w)).L(agent.R()/tau),
			/* n = */ vector.Unit(w),
		)
	}
	if t > 1 && tr < 0 {
		w := vector.Sub(agent.V(), r.P())
		return domain.RightCircle, *hyperplane.New(
			line.New(r.P(), vector.Unit(w)).L(agent.R()/tau),
			vector.Unit(w),
		)
	}

	// Get the distances to the three characteristic lines.
	//
	// TODO(minkezhang): Explicitly test the case where t < 0 || t > 1 and
	// we never follow the domain.Line branch.
	d := map[bool]float64{
		true:  math.Inf(1),
		false: s.S().L().Distance(agent.V()),
	}[oblique || t < 0 || t > 1]
	dl := map[bool]float64{
		true:  math.Inf(1),
		false: l.Distance(agent.V()),
	}[tl < 0]
	dr := map[bool]float64{
		true:  math.Inf(1),
		false: r.Distance(agent.V()),
	}[tr < 0]

	if d <= dl && d <= dr {
		w := vector.Sub(agent.V(), s.S().L().L(t))
		return domain.Line, *hyperplane.New(
			/* p = */ line.New(l.P(), vector.Unit(w)).L(agent.R()/tau),
			/* n = */ vector.Unit(w),
		)
	}

	if dl <= dr {
		w := vector.Sub(agent.V(), l.L(tl))
		return domain.Left, *hyperplane.New(
			/* p = */ line.New(l.P(), vector.Unit(w)).L(agent.R()/tau),
			/* n = */ vector.Unit(w),
		)
	}

	w := vector.Sub(agent.V(), r.L(tl))
	return domain.Right, *hyperplane.New(
		/* p = */ line.New(r.P(), vector.Unit(w)).L(agent.R()/tau),
		/* n = */ vector.Unit(w),
	)
}

func (vo VO) ORCA(agent agent.A, tau float64) hyperplane.HP {
	_, hp := vo.orca(agent, tau)
	return hp
}
