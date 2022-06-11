package cache

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/agent"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"
)

const (
	delta = 1e-2
)

func cache(s segment.S, p vector.V, v vector.V) C {
	return *New(
		s,
		*agent.New(
			agent.O{
				P: p,
				R: 1.0,
				V: v,
			},
		),
		/* tau = */ 1,
	)
}

func TestORCA(t *testing.T) {
	type config struct {
		name string
		c    C
		want hyperplane.HP
	}

	testConfigs := append(
		[]config{},
		func() []config {
			// s is a horizontal line segment spanning (-2, 2) to
			// (2, 2).
			s := *segment.New(
				*line.New(
					*vector.New(-2, 2),
					*vector.New(1, 0),
				),
				0,
				4,
			)
			return []config{
				{
					name: "Line/Top",
					c: cache(
						s,
						/* p = */ *vector.New(0, 4),
						/* v = */ *vector.New(0, 0),
					),
					want: *hyperplane.New(
						*vector.New(0, -1),
						*vector.New(0, 1),
					),
				},
				{
					name: "Line/Bottom",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(0, 0),
					),
					want: *hyperplane.New(
						*vector.New(0, 1),
						*vector.New(0, -1),
					),
				},
				{
					name: "Collision/Top",
					c: cache(
						s,
						*vector.New(0, 2.5),
						*vector.New(1, 1),
					),
					want: *hyperplane.New(
						*vector.New(0, 0),
						*vector.New(0, 1),
					),
				},
				{
					name: "Collision/Bottom",
					c: cache(
						s,
						*vector.New(0, 1.5),
						*vector.New(1, 1),
					),
					want: *hyperplane.New(
						*vector.New(0, 0),
						*vector.New(0, -1),
					),
				},
				// In the case that an agent's center just
				// touches the side of a line segment, we want
				// to ensure the agent does not move towards the
				// line segment.
				{
					name: "Collision/Left/Edge",
					c: cache(
						s,
						*vector.New(-2, 3.1),
						*vector.New(0, 0),
					),
					want: *hyperplane.New(
						*vector.New(0, -0.1),
						*vector.New(0, 1),
					),
				},
				// In the case that an agent's center overlaps
				// the actual line segment, we want to ensure
				// the agent moves away in a reasonable
				// direction.
				{
					name: "Collision/Left",
					c: cache(
						s,
						*vector.New(-2.1, 2),
						*vector.New(0, 1),
					),
					want: *hyperplane.New(
						*vector.New(-899.9500037496875, 8.999500037496867),
						*vector.New(-0.9999500037496875, 0.009999500037496866),
					),
				},
				{
					name: "Collision/Right",
					c: cache(
						s,
						*vector.New(2.1, 2),
						*vector.New(0, 1),
					),
					want: *hyperplane.New(
						*vector.New(899.9500037496875, 8.999500037496867),
						*vector.New(0.9999500037496875, 0.009999500037496866),
					),
				},
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.c.ORCA(); !hyperplane.Within(got, c.want) {
				t.Errorf("ORCA() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestDomain(t *testing.T) {
	type config struct {
		name string
		c    C
		want domain.D
	}

	testConfigs := append(
		[]config{},
		func() []config {
			// s is a horizontal line segment spanning (-1, 1) to
			// (1, 1).
			s := *segment.New(
				*line.New(
					*vector.New(-1, 1),
					*vector.New(1, 0),
				),
				0,
				2,
			)
			return []config{
				{
					name: "Collision/Left",
					c: cache(
						s,
						/* p = */ *vector.New(-2, 1),
						/* v = */ *vector.New(0, 0),
					),
					want: domain.CollisionLeft,
				},
				{
					name: "Collision/Left/Embedded",
					c: cache(
						s,
						*vector.New(-1, 1),
						*vector.New(0, 0),
					),
					want: domain.CollisionLeft,
				},
				{
					name: "Collision/Right",
					c: cache(
						s,
						*vector.New(2, 1),
						*vector.New(0, 0),
					),
					want: domain.CollisionRight,
				},
				{
					name: "Collision/Right/Embedded",
					c: cache(
						s,
						*vector.New(1, 1),
						*vector.New(0, 0),
					),
					want: domain.CollisionRight,
				},
				{
					name: "Collision/Line/Top",
					c: cache(
						s,
						*vector.New(0, 2),
						*vector.New(0, 0),
					),
					want: domain.CollisionLine,
				},
				{
					name: "Collision/Line/Bottom",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(0, 0),
					),
					want: domain.CollisionLine,
				},
				{
					name: "Collision/Line/Embedded",
					c: cache(
						s,
						*vector.New(0, 1),
						*vector.New(0, 0),
					),
					want: domain.CollisionLine,
				},
			}
		}()...,
	)

	testConfigs = append(
		testConfigs,
		func() []config {
			// s is a horizontal line segment spanning (-2, 2) to
			// (2, 2).
			//
			// If the agent is positioned at the origin (0, 0), then
			// we define the tangent vectors
			//
			//   L ~ *vector.New(
			//     -math.Sqrt(7) - 1,
			//     -1 + math.Sqrt(7),
			//   )
			//   R ~ *vector.New(
			//     -math.Sqrt(7) - 1,
			//     1 - math.Sqrt(7),
			//   )
			//
			// For the domain check, we use a line segment with base
			// at the ends of the characteristic segment
			//
			//   L' ~ *line.New(*vector.New(-2, 2), L)
			//   R' ~ *line.New(*vector.New(2, 2), R)
			//   S := *segment.New(
			//     *line.New(
			//       *vector.New(2, 2),
			//       *vector.New(-1, 0),
			//     ),
			//     0,
			//     4,
			//   )
			//
			// Finally, in order to find the boundary line between
			// the left and line regions, we define the bisections
			//
			//   L'' ~ *line.New(
			//     *vector.New(-2, 2),
			//     vector.Unit(L) - vector.Unit(S.D()),
			//   )
			//   R'' ~ *line.New(
			//     *vector.New(2, 2),
			//     vector.Unit(S.D()) - *vector.Unit(R),
			//   )
			//
			// N.B.: This is only true for the specific example, as
			// we have set up the system to be easily reasonable.
			s := *segment.New(
				*line.New(
					*vector.New(-2, 2),
					*vector.New(1, 0),
				),
				0,
				4,
			)
			l := *vector.New(-1-math.Sqrt(7), -1+math.Sqrt(7))
			r := *vector.New(-1-math.Sqrt(7), 1-math.Sqrt(7))
			lpp := *line.New(
				s.L().L(s.TMin()),
				vector.Unit(
					vector.Sub(
						vector.Unit(l),
						vector.Unit(vector.Scale(-1, s.L().D())),
					),
				),
			)
			rpp := *line.New(
				s.L().L(s.TMax()),
				vector.Unit(
					vector.Sub(
						vector.Unit(vector.Scale(-1, s.L().D())),
						vector.Unit(r),
					),
				),
			)

			return []config{
				{
					name: "Region=3",
					c: cache(
						s,
						/* p = */ *vector.New(0, 0),
						/* v = */ *vector.New(0, 4),
					),
					want: domain.Line,
				},
				{
					name: "Region=3/Border=4",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							// Move the resultant
							// velocity a bit
							// downwards to be
							// firmly in region 3.
							*vector.New(0, -delta),
							vector.Add(
								s.L().L(s.TMax()),
								// Ensure the
								// resultant
								// velocity does
								// not point
								// past the
								// midpoint of S
								// (i.e.  cross
								// the Y-axis in
								// this
								// example).
								vector.Scale(1.5, rpp.D()),
							),
						),
					),
					want: domain.Line,
				},
				{
					name: "Region=4/Border=3",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							// Move the resultant
							// velocity a bit
							// upwards to be firmly
							// in region 4.
							*vector.New(0, delta),
							vector.Add(
								s.L().L(s.TMax()),
								vector.Scale(1.5, rpp.D()),
							),
						),
					),
					want: domain.Right,
				},
				{
					name: "Region=3/Border=2",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(0, -delta),
							vector.Add(
								s.L().L(s.TMin()),
								vector.Scale(1.5, lpp.D()),
							),
						),
					),
					want: domain.Line,
				},
				{
					name: "Region=2/Border=3",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(0, delta),
							vector.Add(
								s.L().L(s.TMin()),
								vector.Scale(1.5, lpp.D()),
							),
						),
					),
					want: domain.Left,
				},
				{
					name: "Region=6",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(0, 0),
					),
					want: domain.Line,
				},
				{
					name: "Region=6/Border=1",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(-2, 0),
					),
					want: domain.Line,
				},
				{
					name: "Region=1/Border=6",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(-delta, 0),
							*vector.New(-2, 0),
						),
					),
					want: domain.Left,
				},
				{
					name: "Region=6/Border=5",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(2, 0),
					),
					want: domain.Line,
				},
				{
					name: "Region=5/Border=6",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(delta, 0),
							*vector.New(2, 0),
						),
					),
					want: domain.Right,
				},
				{
					name: "Region=1/Border=2",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(-delta, 0),
							*vector.New(-2, 100),
						),
					),
					want: domain.Left,
				},
				{
					name: "Region=4/Border=5",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(-delta, 0),
							*vector.New(2, 100),
						),
					),
					want: domain.Right,
				},
				{
					name: "Region=5/Border=4",
					c: cache(
						s,
						*vector.New(0, 0),
						vector.Add(
							*vector.New(delta, 0),
							*vector.New(2, 100),
						),
					),
					want: domain.Right,
				},
				{
					name: "Region=2/Border=4",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(-delta, 100),
					),
					want: domain.Left,
				},
				{
					name: "Region=4/Border=2",
					c: cache(
						s,
						*vector.New(0, 0),
						*vector.New(delta, 100),
					),
					want: domain.Right,
				},
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.c.domain(); got != c.want {
				t.Errorf("domain() = %v, want = %v", got, c.want)
			}
		})
	}
}
