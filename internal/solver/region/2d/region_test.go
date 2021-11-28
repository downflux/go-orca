package region

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/line"
	"github.com/google/go-cmp/cmp"

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
			c := *constraint.New(
				*vector.New(1, 0),
				*vector.New(0, 1),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "FirstConstraint",
				c:       c,
				cs:      []constraint.C{},
				success: true,
				want:    *segment.New(l, math.Inf(-1), math.Inf(0)),
			}
		}(),

		func() config {
			c := *constraint.New(
				*vector.New(1, 0),
				*vector.New(1, 0),
			)

			d := *constraint.New(
				*vector.New(-1, 0),
				*vector.New(-1, 0),
			)
			return config{
				name:    "SingleConstraint/Infeasible/Disjoint",
				c:       c,
				cs:      []constraint.C{d},
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

			c := *constraint.New(
				p,
				*vector.New(1, -1),
			)

			d := *constraint.New(
				p,
				*vector.New(1, 0),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "SingleConstraint/Feasible/SetTMin",
				c:       c,
				cs:      []constraint.C{d},
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

			c := *constraint.New(
				p,
				*vector.New(1, 1),
			)

			d := *constraint.New(
				p,
				*vector.New(1, 0),
			)

			l := hyperplane.Line(hyperplane.HP(c))
			return config{
				name:    "SingleConstraint/Feasible/SetTMax",
				c:       c,
				cs:      []constraint.C{d},
				success: true,
				want:    *segment.New(l, math.Inf(-1), 0),
			}
		}(),
	}

	testConfigs = append(
		testConfigs,

		// Assert that intersect() not order-invariant -- it is possible to
		// fail with an infeasibility error in one order, and return a
		// valid segment if the order changes.
		//
		// It is the responsibility of the function calling
		// intersect() to ensure order-invariance.
		//
		// Here, C and D form parallel lines, with both feasibility
		// regions pointing towards the positive X-axis and with the
		// constraint D lying further away from the origin; we will be
		// "relaxing" the constraint if C is added after D, which will
		// return an infeasibility error.
		func() []config {
			c := *constraint.New(
				*vector.New(1, 0),
				*vector.New(1, 0),
			)

			d := *constraint.New(
				*vector.New(2, 0),
				*vector.New(1, 0),
			)

			m := hyperplane.Line(hyperplane.HP(d))
			return []config{
				{
					name:    "SingleConstraint/Infeasible/RelaxedParallelConstraint",
					c:       c,
					cs:      []constraint.C{d},
					success: false,
					want:    segment.S{},
				},
				{
					name:    "SingleConstraint/Feasible/ConstrainedParallelConstraint",
					c:       d,
					cs:      []constraint.C{c},
					success: true,
					want:    *segment.New(m, math.Inf(-1), math.Inf(0)),
				},
			}
		}()...)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := &R{
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
		// Solve a linear optimization problem with constraints given in
		// a simple online problem. See
		// https://www.storyofmathematics.com/linear-programming for
		// more details.
		config{
			name: "Simple",
			m:    Unbounded{},
			cs: []constraint.C{
				// The graph shows a vertical x = 4 constraint,
				// but the solution assumes x = 5.
				*constraint.New(
					*vector.New(5, 0),
					*vector.New(-5, 0),
				),
				*constraint.New(
					*vector.New(0, 4),
					*vector.New(1, 2),
				),
				*constraint.New(
					*vector.New(0, 4),
					*vector.New(-1, -4),
				),
			},
			// The objective function maximizes 3x + 2y.
			o: func(s segment.S) vector.V {
				min := s.L().L(s.TMin())
				max := s.L().L(s.TMax())
				f := func(v vector.V) float64 {
					return 3*v.X() + 2*v.Y()
				}
				if f(min) > f(max) {
					return min
				}
				return max
			},
			v:       *vector.New(5, math.Inf(0)),
			success: true,
			want:    *vector.New(5, 2.75),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := Solve(c.m, c.cs, c.o, c.v); ok != c.success || !vector.Within(got, c.want) {
				t.Errorf("Solve() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
		})
	}
}
