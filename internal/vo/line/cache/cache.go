package cache

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/line/cache/domain"

	vosegment "github.com/downflux/go-orca/internal/geometry/segment"
	voagent "github.com/downflux/go-orca/internal/vo/agent"
	mock "github.com/downflux/go-orca/internal/vo/line/agent"
)

type C struct {
	// segment represents the physical line segment of the obstacle.
	segment segment.S

	// velocity is the absolute obstacle velocity.
	velocity vector.V

	agent agent.A
	tau   float64
}

func New(s segment.S, v vector.V, a agent.A, tau float64) *C {
	return &C{
		segment:  s,
		velocity: v,
		agent:    a,
		tau:      tau,
	}
}

// domain returns the domain in p-space of interaction between the velocity
// obstacle and
// the agent positions. Here, "left" refers the the end of the characteristic line segment
// which has a minimal parametric t value, and "right" refers to the opposite
// end. Note that this convention does not take into account the relative
// orientation of agent itself.
func (c C) domain() domain.D {
	// t is the projected parametric value along the extended line. We need
	// to detect the case where t extends beyond the segment itself, and
	// seg.T() truncates at the segment endpoints.
	t := c.segment.L().T(c.agent.P())

	// Agent physically collides with the semicircle on the left side of the line
	// segment.
	if t <= c.segment.TMin() && vector.Magnitude(
		c.P(c.segment.TMin()),
	) <= c.agent.R() {
		return domain.CollisionLeft
	}

	// Agent physically collides with the semicircle on the right side of the line
	// segment.
	if t >= c.segment.TMax() && vector.Magnitude(
		c.P(c.segment.TMax()),
	) <= c.agent.R() {
		return domain.CollisionRight
	}

	// d is perpendicular distance between the agent and the line.
	d := c.segment.L().Distance(c.agent.P())

	// Agent physically collides wth the line segment itself.
	if (c.segment.TMin() <= t && t <= c.segment.TMax()) && d <= c.agent.R() {
		return domain.CollisionLine
	}

	// Construct a truncated line segment obstacle in v-space (i.e. where the
	// absolute position does not matter anymore), scaled.
	s := *vosegment.New(c.S(), c.agent.R()/c.tau)

	// TODO(minkezhang): Fix this how we calculate nearest side. Currently
	// is inaccurate.
	l := line.New(s.CL().C().P(), s.L())
	r := line.New(s.CR().C().P(), s.R())

	t = s.S().T(c.V())
	dt := s.S().L().Distance(c.V())
	dl := l.Distance(c.V())
	dr := r.Distance(c.V())

	if t < s.S().TMin() || t > s.S().TMax() {
		dt = math.Inf(1)
	}
	if dt < dl && dt < dr {
		return domain.Line
	}
	if dl < dt && dl < dr {
		return domain.Left
	}
	if dr < dt && dr < dl {
		return domain.Right
	}
	return domain.Line
}

func (c C) ORCA() hyperplane.HP {
	lvo := voagent.New(
		*mock.New(
			c.segment.L().L(c.segment.TMin()),
			c.velocity,
		),
	)
	rvo := voagent.New(
		*mock.New(
			c.segment.L().L(c.segment.TMax()),
			c.velocity,
		),
	)

	switch c.domain() {
	case domain.CollisionLeft:
		return lvo.ORCA(c.agent, c.tau)
	case domain.CollisionRight:
		return rvo.ORCA(c.agent, c.tau)
	case domain.CollisionLine:
		return *hyperplane.New(
			c.S().L().P(),
			*vector.New(
				c.S().L().N().Y(),
				-c.S().L().N().X(),
			),
		)
	}
	panic("unimplemented case")
}

// S returns the characteristic line segment defining the velocity obstacle,
// taking into account the time scalar 𝜏.
func (c C) S() segment.S { return s(c.segment, c.agent, c.tau) }

func (c C) V() vector.V { return v(c.velocity, c.agent) }

// P returns the position vector in p-space from the agent to a specific point
// along the velocity obstacle line. Note that the distance here is independent
// of the time scaling factor 𝜏.
func (c C) P(t float64) vector.V { return p(c.segment, c.agent, t) }

// P calculates the relative physical position from the line obstacle to the
// agent position.
//
// The input parameter t is the parametric value along the line segment.
//
// The returned relative position vector points away from the line segment.
//
// N.B.: The position does not scale with the time factor 𝜏.
func p(s segment.S, a agent.A, t float64) vector.V {
	return vector.Sub(s.L().L(t), a.P())
}

// v returns the relative velocity between the agent and the obstacle line.
func v(v vector.V, a agent.A) vector.V { return vector.Sub(a.V(), v) }

// s generates a scaled line segment based on the lookahead time and the agent.
func s(s segment.S, a agent.A, tau float64) segment.S {
	return *segment.New(
		*line.New(
			vector.Scale(1/tau, vector.Sub(s.L().P(), a.P())),
			vector.Scale(1/tau, s.L().D()),
		),
		s.TMin(),
		s.TMax(),
	)
}
