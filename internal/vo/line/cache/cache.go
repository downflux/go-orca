package cache

import (
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/line/domain"
)

type C struct {
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

func (c C) domain() domain.D {
	// ld is the distance from the relative velocity between the line
	// obstacle and the agent to the VO cutoff line.
	ld := c.S().L().Distance(c.V())

	// t is the projected parametric value along the extended line. We need
	// to detect the case where t extends beyond the segment itself, and
	// seg.T() truncates at the segment endpoints.
	t := c.S().L().T(c.V())

	// Agent collides with the semicircle on the left side of the line
	// segment.
	if t <= c.S().TMin() && vector.Magnitude(
		vector.Sub(
			c.V(),
			c.S().L().L(c.S().TMin()),
		),
	) <= c.agent.R() {
		return domain.CollisionLeft
	}

	// Agent collides with the semicircle on the right side of the line
	// segment.
	if t >= c.S().TMax() && vector.Magnitude(
		vector.Sub(
			c.V(),
			c.S().L().L(c.S().TMax()),
		),
	) <= c.agent.R() {
		return domain.CollisionRight
	}

	// Agent collides wth the line segment itself.
	if (c.S().TMin() <= t && t <= c.S().TMax()) && ld <= c.agent.R() {
		return domain.CollisionLine
	}

	panic("unimplemented domain")
}

func (c C) S() segment.S { return s(c.segment, c.agent, c.tau) }
func (c C) V() vector.V  { return v(c.velocity, c.agent) }

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
