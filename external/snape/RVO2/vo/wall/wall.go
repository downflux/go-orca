package wall

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
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
	d := vo.obstacle.L().Distance(agent.P())
	// Check for collisions.
	t := vo.obstacle.L().T(agent.P())
	if t <= vo.obstacle.TMin() && d1 <= agent.R() {
		return domain.CollisionLeft, *hyperplane.New(
			*vector.New(0, 0), // VOpt
			vector.Unit(*vector.New(-rp1.X(), -rp1.Y())),
		)

	}
	if t >= vo.obstacle.TMax() && d2 <= agent.R() {
		return domain.CollisionRight, *hyperplane.New(
			*vector.New(0, 0),
			vector.Unit(*vector.New(-rp2.X(), -rp2.Y())),
		)
	}
	if vo.obstacle.TMin() <= t && t <= vo.obstacle.TMax() && d <= agent.R() {
		return domain.CollisionLine, *hyperplane.New(
			*vector.New(0, 0),
			// TODO(minkezhang): Handle the NaN case when the agent
			// lies on the line.
			vector.Unit(
				vector.Sub(
					agent.P(),
					vo.obstacle.L().L(t),
				),
			),
		)
	}

	// o is the truncated base of the VO cone in v-space.
	o := *segment.New(
		*line.New(
			vector.Scale(1/tau, vector.Sub(
				vo.obstacle.L().P(),
				agent.P())),
			vector.Scale(1/tau, vo.obstacle.L().D())),
		vo.obstacle.TMin(),
		vo.obstacle.TMax(),
	)
	vosegment, err := vosegment.New(o, *vector.New(0, 0), agent.R()/tau)
	if err != nil {
		panic(fmt.Sprintf("error while constructing the truncated VO cone base: %v", err))
	}

	l := *line.New(
		vosegment.S().L().L(vosegment.S().TMin()),
		vosegment.L().D())
	r := *line.New(
		vosegment.S().L().L(vosegment.S().TMax()),
		vosegment.R().D())

	// TODO(minkezhang): Test oblique case.
	oblique := epsilon.Within(vosegment.S().TMin(), vosegment.S().TMax())

	// t is the projected parametric value along the truncated base. Note
	// that the segment reported by vosegment.S() is normalized, so t is always
	// in [0, 1]. In the oblique case, the line segment is not well-defined,
	// so we use a placeholder value here.
	t = 0.5
	if !oblique {
		t = vosegment.S().L().T(agent.V())
	}
	tl := l.T(agent.V())
	tr := r.T(agent.V())

	// Note that l is always directed away towards the origin, while r is
	// always directed away. RVO2 assumes both l and r are directed away
	// from the origin.
	if (t < 0 && tl > 0) || (oblique && tl > 0 && tr < 0) {
		// LeftCircle
		w := vector.Sub(agent.V(), vosegment.S().L().L(vosegment.S().TMin()))
		return domain.Left, *hyperplane.New(
			// Hyperplane lies tangent to the VO object.
			/* p = */
			line.New(
				vosegment.S().L().L(vosegment.S().TMin()),
				vector.Unit(w),
			).L(agent.R()/tau),
			/* n = */ vector.Unit(w),
		)
	}
	if t > 1 && tr < 0 {
		// RightCircle
		w := vector.Sub(agent.V(), vosegment.S().L().L(vosegment.S().TMax()))
		return domain.Right, *hyperplane.New(
			line.New(
				vosegment.S().L().L(vosegment.S().TMax()),
				vector.Unit(w),
			).L(agent.R()/tau),
			vector.Unit(w),
		)
	}

	// Get the distances to the three characteristic lines.
	d = math.Inf(1)
	if !oblique {
		d = vosegment.S().L().Distance(agent.V())
	}

	dl := math.Inf(1)
	if tl < 0 {
		dl = l.Distance(agent.V())
	}

	dr := math.Inf(1)
	if tr > 0 {
		dr = r.Distance(agent.V())
	}

	data, _ := json.MarshalIndent(map[string]interface{}{
		"l": fmt.Sprintf("P == %v, D == %v", l.P(), l.D()),
		"r": fmt.Sprintf("P == %v, D == %v", r.P(), r.D()),
		"s": fmt.Sprintf(
			"P == %v, D == %v, TMin == %v, TMax == %v",
			vosegment.S().L().P(),
			vosegment.S().L().D(),
			vosegment.S().TMin(),
			vosegment.S().TMax()),
		"tl": tl,
		"tr": tr,
		"t":  t,
	}, "", "  ")
	fmt.Printf("DEBUG(mock): %s\n", data)

	if d <= dl && d <= dr {
		w := vector.Sub(agent.V(), vosegment.S().L().L(t))
		return domain.Line, *hyperplane.New(
			/* p = */ line.New(
				vosegment.S().L().L(vosegment.S().TMin()),
				vector.Unit(w),
			).L(agent.R()/tau),
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
	domain, orca := vo.orca(agent, tau)
	fmt.Printf("DEBUG(mock): domain == %v\n", domain.String())
	return orca
}
