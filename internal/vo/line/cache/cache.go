package cache

import (
	"fmt"
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/line/cache/domain"

	vosegment "github.com/downflux/go-orca/internal/geometry/segment"
	voagent "github.com/downflux/go-orca/internal/vo/agent"
	mock "github.com/downflux/go-orca/internal/vo/line/agent"
)

const (
	minTau = 1e-3
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
// obstacle and the agent positions.
//
// Obstacle "left" and "right" domains are hard to consruct ahead of time; when
// referring to the collision domains, "left" refers to the end of the
// characteristic line segment which has a minimal parametric t value, while in
// the non-collision domains, the characteristic line segment may be flipped to
// preserve normal orientation between the three lines. Note that this
// convention does not take into account the relative orientation of agent
// itself.
func (c C) domain() domain.D {

	// t is the projected parametric value along the extended line. We need
	// to detect the case where t extends beyond the segment itself, and
	// seg.T() truncates at the segment endpoints.
	t := c.segment.L().T(c.agent.P())

	// Agent physically collides with the semicircle on the left side of the
	// line segment.
	//
	// A physical collision means that the agent overlaps the segment --
	// becasue of computational limits, we split the check for
	//
	//   || c.segment.L().L(s.TMin()) - c.agent.P() || <= c.agent.R()
	//
	// into a strict inequality check and a float equality check (i.e.
	// epsilon.Within()). If we do not do this check, the VO segment
	// constructor may raise an unexpected error.
	if p := vector.Magnitude(c.P(c.segment.TMin())); t <= c.segment.TMin() && (p < c.agent.R() || epsilon.Within(p, c.agent.R())) {
		return domain.CollisionLeft
	}

	// Agent physically collides with the semicircle on the right side of
	// the line segment.
	if p := vector.Magnitude(c.P(c.segment.TMax())); t >= c.segment.TMax() && (p < c.agent.R() || epsilon.Within(p, c.agent.R())) {
		return domain.CollisionRight
	}

	// d is perpendicular distance between the agent and the line.
	d := c.segment.L().Distance(c.agent.P())

	// Agent physically collides wth the line segment itself.
	if (c.segment.TMin() <= t && t <= c.segment.TMax()) && (d < c.agent.R() || epsilon.Within(d, c.agent.R())) {
		return domain.CollisionLine
	}

	// Construct a truncated line segment obstacle in v-space (i.e. where
	// the absolute position does not matter anymore), scaled.
	s := *vosegment.New(c.S(), c.agent.R()/c.tau)

	// If the agent does not physically collide with the obstacle in
	// p-space, we need to determine if the agent will collide with the line
	// in the future -- that is, if the agent and line obstacles will
	// collide in v-space. We can split the VO into five separate domains,
	// per the official RVO2 implementation, as follows
	//
	//    \     |     /
	//   L \ 2 /3\ 4 / R
	//   1  \ /___\ /  5
	//       |  S  |
	//       |  6  |
	//
	// Here, L, R are directed in the same direction as the tangential lines
	// of the VO, but originate from the bottom line segment S, instead of
	// at the tangential points. We use this construction to define regions
	// 2 and 4, which are points in v-space that are closer to L and R than
	// S. Region 6 is defined by convenience -- we remember that for the
	// perpendicular bounds for this region, the generated normal to the VO
	// is just the normal of the line segment itself.
	//
	// Note that the boundary lines opposite L and R, defining regions 2 and
	// 4, respectively, bisects S.
	//
	// We are modifying the six regions above to a slightly altered version
	// for ease of calculations
	//
	//    \1 |2 | 4| 5/
	//   L \ | /3\ | / R
	//   1  \|/___\|/  5
	//       |  S  |
	//       |  6  |
	//
	// These regions are the ones mentioned hereafter and in the tests.

	l := line.New(s.CL().C().P(), s.L())
	r := line.New(s.CR().C().P(), s.R())

	t = s.S().L().T(c.V())

	// The characteristic segment s may be oriented in either direction
	// relative to the "left" and "right" tangent lines with respect to the
	// parametric value t = 0; thus, we need to check the underlying segment
	// construction to determine which direction s is pointing, and use that
	// to determine what domain we are in.
	if t < s.S().TMin() {
		return map[bool]domain.D{
			// Covers region 1.
			true: domain.Left,
			// Covers region 5.
			false: domain.Right,
		}[s.IsLeftNegative()]
	}
	if t > s.S().TMax() {
		return map[bool]domain.D{
			true:  domain.Right,
			false: domain.Left,
		}[s.IsLeftNegative()]
	}

	// We know that t is bounded between the min and max t-values of S by
	// now. The official implementation uses w to calculate the distance to
	// the line segments; however, we note that this distance may be
	// visually represented by measuring the distance between v and the
	// perpendicular distance to the segments, which can be reasoned by
	// drawing a diagram; note that w is important for relating to the VO
	// radius, but here, we are deferring the circular domain calculations
	// to the cone VO objects themselves.
	tl := l.T(c.V())
	tr := r.T(c.V())

	d = s.S().L().Distance(c.V())
	dl := l.Distance(c.V())
	dr := r.Distance(c.V())

	// This check is for region 7 and parts of region 3 (specifically, the
	// parts "under" region 3 bounded by the tl = 0 and tr = 0 normal lines.
	if s.IsLeftNegative() && (tl > 0 && tr < 0) || !s.IsLeftNegative() && (tl < 0 && tr > 0) {
		return domain.Line
	}

	md := min([]float64{d, dl, dr})
	return map[float64]domain.D{
		d:  domain.Line,
		dl: domain.Left,
		dr: domain.Right,
	}[md]
}

func (c C) ORCA() hyperplane.HP {
	switch d := c.domain(); d {
	case domain.CollisionLeft:
		return voagent.New(
			*mock.New(
				c.segment.L().L(c.segment.TMin()),
				c.velocity,
			),
		).ORCA(c.agent, c.tau)
	case domain.CollisionRight:
		return voagent.New(
			*mock.New(
				c.segment.L().L(c.segment.TMax()),
				c.velocity,
			),
		).ORCA(c.agent, c.tau)
	case domain.Left:
		s := *vosegment.New(c.segment, c.agent.R())
		return voagent.New(
			mock.New(
				s.CL().C().P(),
				c.velocity,
			),
		).ORCA(c.agent, c.tau)
	case domain.Right:
		s := *vosegment.New(c.segment, c.agent.R())
		return voagent.New(
			mock.New(
				s.CR().C().P(),
				c.velocity,
			),
		).ORCA(c.agent, c.tau)
	case domain.CollisionLine:
		fallthrough
	case domain.Line:
		tau := c.tau
		// n calculates the normal of the hyperlane pointing into the
		// feasible region. Note that u points away from the VO segment,
		// so for non-collision cases, we need to flip the orientation
		// of the normal.
		orientation := -1.0

		if d == domain.CollisionLine {
			tau = minTau
			orientation = 1.
		}

		// tau could be minTau; we cannot assume c.S() is valid here.
		s := s(c.segment, c.agent, tau)
		w := vector.Sub(
			c.agent.V(),
			s.L().L(s.T(c.agent.V())),
		)
		r := c.agent.R() / tau
		u := vector.Scale(
			r/vector.Magnitude(w)-1,
			w,
		)
		n := vector.Scale(orientation, vector.Unit(u))

		return *hyperplane.New(
			vector.Add(c.agent.V(), vector.Scale(0.5, u)),
			n,
		)
	default:
		panic(fmt.Sprintf("invalid VO projection %v", d))
	}
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
// The returned relative position vector points towards the line segment.
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
			vector.Scale(1/tau, p(s, a, s.TMin())),
			vector.Scale(1/tau, s.L().D()),
		),
		s.TMin(),
		s.TMax(),
	)
}

func min(vs []float64) float64 {
	m := math.Inf(1)
	for _, v := range vs {
		if v < m {
			m = v
		}
	}
	return m
}
