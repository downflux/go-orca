package solver

import (
	"fmt"
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/line"
	"github.com/downflux/go-orca/internal/geometry/2d/constraint"
	"github.com/google/go-cmp/cmp"

	c2d "github.com/downflux/go-geometry/2d/constraint"
	l2d "github.com/downflux/go-geometry/2d/line"
)

var _ M = Unbounded{}

func TestIntersect(t *testing.T) {
	type config struct {
		name    string
		c       constraint.C
		cs      []constraint.C
		success bool
		want    segment.S
	}

	testConfigs := []config{

		func() config {
			c := *c2d.New(
				*vector.New(1, 0),
				*vector.New(0, 1),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "FirstConstraint",
				c:       *constraint.New(c, true),
				cs:      []constraint.C{},
				success: true,
				want:    *segment.New(l, math.Inf(-1), math.Inf(0)),
			}
		}(),

		func() config {
			c := *c2d.New(
				*vector.New(0, 1),
				*vector.New(0, 1),
			)

			d := *c2d.New(
				*vector.New(0, -1),
				*vector.New(0, -1),
			)
			return config{
				name:    "SingleConstraint/Infeasible/Disjoint",
				c:       *constraint.New(c, true),
				cs:      []constraint.C{*constraint.New(d, true)},
				success: false,
				want:    segment.S{},
			}
		}(),

		// Check if we are overriding S.TMin() when appropriate.
		//
		// TMin is set if the direction of the new constraint C.D() lies
		// in the existing constraint's feasiblity region F(D).
		func() config {
			p := *vector.New(1, 0)

			c := *c2d.New(
				p,
				*vector.New(1, -1),
			)

			d := *c2d.New(
				p,
				*vector.New(1, 0),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "SingleConstraint/Feasible/SetTMin",
				c:       *constraint.New(c, true),
				cs:      []constraint.C{*constraint.New(d, true)},
				success: true,
				want:    *segment.New(l, 0, math.Inf(0)),
			}
		}(),

		// Check if we are overriding S.TMax() when appropriate.
		//
		// TMax is set if the direction of the new constraint C.D() lies
		// outside the existing constraint's feasiblity region F(D).
		func() config {
			p := *vector.New(1, 0)

			c := *c2d.New(
				p,
				*vector.New(1, 1),
			)

			d := *c2d.New(
				p,
				*vector.New(1, 0),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "SingleConstraint/Feasible/SetTMax",
				c:       *constraint.New(c, true),
				cs:      []constraint.C{*constraint.New(d, true)},
				success: true,
				want:    *segment.New(l, math.Inf(-1), 0),
			}
		}(),
	}

	testConfigs = append(
		testConfigs,

		// Assert that intersect() not order-invariant -- it is possible
		// to fail with an infeasibility error in one order, and return
		// a valid segment if the order changes.
		//
		// It is the responsibility of the function calling intersect()
		// to ensure order-invariance.
		//
		// Here, C and D form parallel lines, with both feasibility
		// regions pointing towards the positive X-axis and with the
		// constraint D lying further away from the origin; we will be
		// "relaxing" the constraint if C is added after D, which will
		// return an infeasibility error.
		func() []config {
			c := *c2d.New(
				*vector.New(1, 0),
				*vector.New(1, 0),
			)

			d := *c2d.New(
				*vector.New(2, 0),
				*vector.New(1, 0),
			)

			m := hyperplane.Line(hyperplane.HP(d))
			return []config{
				{
					name:    "SingleConstraint/Infeasible/RelaxedParallelConstraint",
					c:       *constraint.New(c, true),
					cs:      []constraint.C{*constraint.New(d, true)},
					success: false,
					want:    segment.S{},
				},
				{
					name:    "SingleConstraint/Feasible/ConstrainedParallelConstraint",
					c:       *constraint.New(d, true),
					cs:      []constraint.C{*constraint.New(c, true)},
					success: true,
					want:    *segment.New(m, math.Inf(-1), math.Inf(0)),
				},
			}
		}()...)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := &region{
				m:           Unbounded{},
				constraints: c.cs,
			}
			got, ok := r.intersect(c.c)
			if ok != c.success {
				t.Errorf("intersect() = _, %v, want = _, %v", ok, c.success)
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					segment.S{},
					l2d.L{},
					line.L{})); diff != "" {
				t.Errorf("intersect() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestSolve(t *testing.T) {
	type config struct {
		name    string
		m       M
		cs      []constraint.C
		o       O
		v       vector.V
		success bool
		want    vector.V
	}

	testConfigs := []config{
		{
			name: "Infeasible",
			m:    Unbounded{},
			cs: []constraint.C{
				*constraint.New(
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
					true,
				),
				*constraint.New(
					*c2d.New(
						*vector.New(0, -1),
						*vector.New(0, -1),
					),
					true,
				),
			},
			o:       func(segment.S) vector.V { return vector.V{} },
			v:       *vector.New(0, 1),
			success: false,
		},
	}

	testConfigs = append(
		testConfigs,
		// Solve a linear optimization problem with constraints given in
		// a simple online problem. See
		// https://www.storyofmathematics.com/linear-programming for
		// more details.
		func() []config {
			cs := []constraint.C{
				// The graph shows a vertical x = 4 constraint,
				// but the solution assumes x = 5.
				*constraint.New(
					*c2d.New(
						*vector.New(5, 0),
						*vector.New(-5, 0),
					),
					true,
				),
				*constraint.New(
					*c2d.New(
						*vector.New(0, 4),
						*vector.New(1, 2),
					),
					true,
				),
				*constraint.New(
					*c2d.New(
						*vector.New(0, 4),
						*vector.New(-1, -4),
					),
					true,
				),
			}
			o := func(s segment.S) vector.V {
				min := s.L().L(s.TMin())
				max := s.L().L(s.TMax())
				f := func(v vector.V) float64 {
					return 3*v.X() + 2*v.Y()
				}
				if f(min) > f(max) {
					return min
				}
				return max
			}

			// The objective function maximizes 3x + 2y.  Our
			// iniital solution must lie on the "corner" of our
			// bounding constraint m; since m is unbounded, we
			// choose an arbitrary corner on the extended reals.
			vs := []vector.V{
				*vector.New(math.Inf(0), math.Inf(0)),
				*vector.New(math.Inf(-1), math.Inf(0)),
				*vector.New(math.Inf(-1), math.Inf(-1)),
				*vector.New(math.Inf(0), math.Inf(-1)),
			}

			var testConfigs []config
			for _, v := range vs {
				testConfigs = append(testConfigs, config{
					name:    fmt.Sprintf("Simple/v0=%v", v),
					m:       Unbounded{},
					cs:      cs,
					o:       o,
					v:       v,
					success: true,
					// This is the given solution and is a
					// constant.
					want: *vector.New(5, 2.75),
				})
			}
			return testConfigs
		}()...)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := Solve(c.m, c.cs, c.o, c.v); ok != c.success || !vector.Within(got, c.want) {
				t.Errorf("Solve() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
		})
	}
}
