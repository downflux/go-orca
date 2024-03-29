// Package cache implements the VO for a wall segment obstacle.
//
// TODO(minkezhang): Ensure VO returned is continuous at the regional
// boundaries.
package cache

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/agent/opt"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"

	agentimpl "github.com/downflux/go-orca/internal/agent"
	vosegment "github.com/downflux/go-orca/internal/geometry/2d/segment"
	voagent "github.com/downflux/go-orca/internal/vo/agent"
)

const (
	minTau = 1e-3
)

type C struct {
	// segment represents the physical line segment of the obstacle.
	segment segment.S

	agent agent.A
	tau   float64
}

func New(s segment.S, a agent.A, tau float64) *C {
	return &C{
		segment: s,
		agent:   a,
		tau:     tau,
	}
}

func (c C) orca() (domain.D, hyperplane.HP) {
	// Per van den Berg et al. (2011), we expect VOpt to be
	// the 0-vector, and that u lies directly on the tangent
	// plane (i.e. opt.WeightNone). This means the agent
	// will not attempt to avoid collisions with the wall
	// unless this will result in a collision within the
	// next timestep.
	o := opt.O{
		Weight: opt.WeightNone,
		VOpt:   opt.VOptZero,
	}

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
		return domain.CollisionLeft, voagent.New(
			agentimpl.New(
				agentimpl.O{
					P: c.segment.L().L(c.segment.TMin()),
					V: *vector.New(0, 0),
				},
			),
			o,
		).ORCA(c.agent, c.tau)
	}

	// Agent physically collides with the semicircle on the right side of
	// the line segment.
	if p := vector.Magnitude(c.P(c.segment.TMax())); t >= c.segment.TMax() && (p < c.agent.R() || epsilon.Within(p, c.agent.R())) {
		return domain.CollisionRight, voagent.New(
			agentimpl.New(
				agentimpl.O{
					P: c.segment.L().L(c.segment.TMax()),
					V: *vector.New(0, 0),
				},
			),
			o,
		).ORCA(c.agent, c.tau)
	}

	// d is perpendicular distance between the agent and the line.
	d := c.segment.L().Distance(c.agent.P())

	// Agent physically collides wth the line segment itself.
	if (c.segment.TMin() <= t && t <= c.segment.TMax()) && (d < c.agent.R() || epsilon.Within(d, c.agent.R())) {
		n := vector.Unit(
			vector.Sub(
				c.agent.P(),
				c.segment.L().L(c.segment.T(c.agent.P())),
			),
		)
		return domain.CollisionLine, *hyperplane.New(opt.VOptZero(c.agent), n)
	}

	// Construct a truncated line segment obstacle in v-space (i.e. where
	// the absolute position does not matter anymore), scaled.
	s := *vosegment.New(c.S(), *vector.New(0, 0), c.agent.R()/c.tau)

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

	l := line.New(s.S().L().L(s.S().TMax()), s.L().D())
	r := line.New(s.S().L().L(s.S().TMin()), s.R().D())

	oblique := vector.Within(
		s.S().L().L(s.S().TMin()),
		s.S().L().L(s.S().TMax()),
	)
	t = (s.S().TMax() - s.S().TMin()) / 2
	if !oblique {
		t = s.S().L().T(c.agent.V())
	}

	// We know that t is bounded between the min and max t-values of S by
	// now. The official implementation uses w to calculate the distance to
	// the line segments; however, we note that this distance may be
	// visually represented by measuring the distance between v and the
	// perpendicular distance to the segments, which can be reasoned by
	// drawing a diagram; note that w is important for relating to the VO
	// radius, but here, we are deferring the circular domain calculations
	// to the cone VO objects themselves.
	tl := l.T(c.agent.V())
	tr := r.T(c.agent.V())

	// S() is a segment which is directed from the right to the left. This
	// is done to ensure a consistent orientation between L(), S(), and R().
	// Because S() starts from the "right" side, we are flipping the t
	// boundary check from the official RVO2 implementation.
	if (t > s.S().TMax() && tl < 0) || (oblique && tl < 0 && tr > 0) {
		w := vector.Sub(c.agent.V(), l.P())
		return domain.LeftCircle, *hyperplane.New(
			/* p = */ line.New(
				l.P(), vector.Unit(w),
			).L(c.agent.R()/c.tau),
			/* n = */ vector.Unit(w),
		)
	}
	if t < s.S().TMin() && tr > 0 {
		w := vector.Sub(c.agent.V(), r.P())
		return domain.RightCircle, *hyperplane.New(
			line.New(
				r.P(), vector.Unit(w),
			).L(c.agent.R()/c.tau),
			vector.Unit(w),
		)
	}

	d = map[bool]float64{
		true:  math.Inf(1),
		false: s.S().L().Distance(c.agent.V()),
	}[oblique || t < s.S().TMin() || t > s.S().TMax()]
	dl := map[bool]float64{
		true:  math.Inf(1),
		false: l.Distance(c.agent.V()),
	}[tl < 0]
	dr := map[bool]float64{
		true:  math.Inf(1),
		false: r.Distance(c.agent.V()),
	}[tr > 0]

	// This check is for region 7 and parts of region 3 (specifically, the
	// parts "under" region 3 bounded by the tl = 0 and tr = 0 normal lines.
	if tl < 0 && tr > 0 || d <= dl && d <= dr {
		w := vector.Sub(c.agent.V(), s.S().L().L(t))
		return domain.Line, *hyperplane.New(
			line.New(
				r.P(), vector.Unit(w),
			).L(c.agent.R()/c.tau),
			vector.Unit(w),
		)
	}
	if dl <= dr {
		w := vector.Sub(c.agent.V(), l.L(tl))
		return domain.Left, *hyperplane.New(
			line.New(
				l.P(), vector.Unit(w),
			).L(c.agent.R()/c.tau),
			vector.Unit(w),
		)

	}

	w := vector.Sub(c.agent.V(), r.L(tr))
	return domain.Right, *hyperplane.New(
		line.New(
			r.P(), vector.Unit(w),
		).L(c.agent.R()/c.tau),
		vector.Unit(w),
	)
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
	d, _ := c.orca()
	return d
}

func (c C) ORCA() hyperplane.HP {
	_, hp := c.orca()
	return hp
}

// S returns the characteristic line segment defining the velocity obstacle,
// taking into account the time scalar 𝜏.
func (c C) S() segment.S { return s(c.segment, c.agent, c.tau) }

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
