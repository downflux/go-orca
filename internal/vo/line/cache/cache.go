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
	if t <= c.segment.TMin() && vector.Magnitude(
		c.P(c.segment.TMin()),
	) <= c.agent.R() {
		return domain.CollisionLeft
	}

	// Agent physically collides with the semicircle on the right side of
	// the line segment.
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
	// Complicating this calculation is the fact that we do not enforce a
	// convention of directionality for L, S, or R directly -- that is, S
	// here may point either to the left or right. We do enforce that L, S,
	// and R are normally oriented with one another, i.e.
	//
	//   |L x S| > 0, and
	//   |S x R| > 0
	//
	// This means the line segments defining the cone are actually directed
	// either as
	//
	//     ____/ R  or  L \____
	//   L \ S              S / R
	//
	// We can check for which orientation we are in by checking which end of
	// the S corresponds with the base of the left line segment L.
	//
	// We (rather arbitrarily) define the "left-negative" orientation as the
	// first case, and "left-positive" as the second.
	//
	// We are modifying the six regions above to a slightly altered version
	// for ease of calculations
	//
	//    \1 |2 | 4|  /
	//   L \ | /3\ | / R
	//   1  \|/___\|/  5
	//       |  S  |
	//       |  6  |
	//
	// These regions are the ones mentioned hereafter and in the tests.

	l := line.New(s.CL().C().P(), s.L())
	r := line.New(s.CR().C().P(), s.R())

	t = s.S().L().T(c.V())

	isLeftNegative := vector.Within(
		s.CL().C().P(),
		s.S().L().L(s.S().TMin()))

	if t < s.S().TMin() {
		return map[bool]domain.D{
			// Covers region 1.
			true: domain.Left,
			// Covers region 5.
			false: domain.Right,
		}[isLeftNegative]
	}
	if t > s.S().TMax() {
		return map[bool]domain.D{
			true:  domain.Right,
			false: domain.Left,
		}[isLeftNegative]
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
	if isLeftNegative && (tl > 0 && tr < 0) || !isLeftNegative && (tl < 0 && tr > 0) {
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
	switch c.domain() {
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
	case domain.CollisionLine:
		// TODO(minkezhang): Fix CollisionLine, Line case to be segment
		// orientation-agnostic.
		//
		// TODO(minkezhang): Move IsLeftNegative into vosegment.S.
		return *hyperplane.New(
			c.S().L().P(),
			*vector.New(
				c.S().L().N().Y(),
				-c.S().L().N().X(),
			),
		)
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

func min(vs []float64) float64 {
	m := math.Inf(1)
	for _, v := range vs {
		if v < m {
			m = v
		}
	}
	return m
}
