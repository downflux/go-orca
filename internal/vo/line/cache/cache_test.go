package cache

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/vo/line/cache/domain"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

const (
	epsilon = 1e-2
)

func TestDomain(t *testing.T) {
	type config struct {
		name string
		c    C
		want domain.D
	}

	f := func(name string, s segment.S, p vector.V, v vector.V, d domain.D) config {
		return config{
			name: name,
			c: *New(
				s,
				/* v = */ *vector.New(0, 0),
				*mock.New(
					mock.O{
						P: p,
						R: 1.0,
						V: v,
					},
				),
				/* tau = */ 1.0,
			),
			want: d,
		}
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
				f(
					"Collision/Left",
					s,
					/* p = */ *vector.New(-2, 1),
					/* v = */ *vector.New(0, 0),
					domain.CollisionLeft,
				),
				f(
					"Collision/Right",
					s,
					*vector.New(2, 1),
					*vector.New(0, 0),
					domain.CollisionRight,
				),
				f(
					"Collision/Top",
					s,
					*vector.New(0, 2),
					*vector.New(0, 0),
					domain.CollisionLine,
				),
				f(
					"Collision/Bottom",
					s,
					*vector.New(0, 0),
					*vector.New(0, 0),
					domain.CollisionLine,
				),
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
				f(
					"Region=3",
					s,
					/* p = */ *vector.New(0, 0),
					/* v = */ *vector.New(0, 4),
					domain.Line,
				),
				f(
					"Region=3/Border=4",
					s,
					*vector.New(0, 0),
					vector.Add(
						// Move the resultant velocity a
						// bit downwards to be firmly in
						// region 3.
						*vector.New(0, -epsilon),
						vector.Add(
							s.L().L(s.TMax()),
							// Ensure the resultant
							// velocity does not
							// point past the
							// midpoint of S (i.e.
							// cross the Y-axis in
							// this example).
							vector.Scale(1.5, rpp.D()),
						),
					),
					domain.Line,
				),
				f(
					"Region=4/Border=3",
					s,
					*vector.New(0, 0),
					vector.Add(
						// Move the resultant velocity a
						// bit upwards to be firmly in
						// region 4.
						*vector.New(0, epsilon),
						vector.Add(
							s.L().L(s.TMax()),
							vector.Scale(1.5, rpp.D()),
						),
					),
					domain.Right,
				),
				f(
					"Region=3/Border=2",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(0, -epsilon),
						vector.Add(
							s.L().L(s.TMin()),
							vector.Scale(1.5, lpp.D()),
						),
					),
					domain.Line,
				),
				f(
					"Region=2/Border=3",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(0, epsilon),
						vector.Add(
							s.L().L(s.TMin()),
							vector.Scale(1.5, lpp.D()),
						),
					),
					domain.Left,
				),
				f(
					"Region=6",
					s,
					*vector.New(0, 0),
					*vector.New(0, 0),
					domain.Line,
				),
				f(
					"Region=6/Border=1",
					s,
					*vector.New(0, 0),
					*vector.New(-2, 0),
					domain.Line,
				),
				f(
					"Region=1/Border=6",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(-epsilon, 0),
						*vector.New(-2, 0),
					),
					domain.Left,
				),
				f(
					"Region=6/Border=5",
					s,
					*vector.New(0, 0),
					*vector.New(2, 0),
					domain.Line,
				),
				f(
					"Region=5/Border=6",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(epsilon, 0),
						*vector.New(2, 0),
					),
					domain.Right,
				),
				f(
					"Region=1/Border=2",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(-epsilon, 0),
						*vector.New(-2, 100),
					),
					domain.Left,
				),
				f(
					"Region=4/Border=5",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(-epsilon, 0),
						*vector.New(2, 100),
					),
					domain.Right,
				),
				f(
					"Region=5/Border=4",
					s,
					*vector.New(0, 0),
					vector.Add(
						*vector.New(epsilon, 0),
						*vector.New(2, 100),
					),
					domain.Right,
				),
				f(
					"Region=2/Border=4",
					s,
					*vector.New(0, 0),
					*vector.New(-epsilon, 100),
					domain.Left,
				),
				f(
					"Region=4/Border=2",
					s,
					*vector.New(0, 0),
					*vector.New(epsilon, 100),
					domain.Right,
				),
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
