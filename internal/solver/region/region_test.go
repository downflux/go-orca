package region

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/line"
	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/segment"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/solver/constraint"
	"github.com/google/go-cmp/cmp"
)

const (
	tolerance = 1e-10
)

func TestAdd(t *testing.T) {
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
				*plane.New(
					*vector.New(1, 0),
					*vector.New(0, 1),
				),
			)

			return config{
				name:    "FirstConstraint",
				c:       c,
				cs:      []constraint.C{},
				success: true,
				want: *segment.New(
					c.HP().L(),
					math.Inf(-1),
					math.Inf(0),
				),
			}
		}(),

		func() config {
			c := *constraint.New(
				*plane.New(
					*vector.New(1, 0),
					*vector.New(1, 0),
				),
			)

			d := *constraint.New(
				*plane.New(
					*vector.New(-1, 0),
					*vector.New(-1, 0),
				),
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
				*plane.New(
					p,
					*vector.New(1, -1),
				),
			)

			d := *constraint.New(
				*plane.New(
					p,
					*vector.New(1, 0),
				),
			)
			return config{
				name:    "SingleConstraint/Feasible/SetTMin",
				c:       c,
				cs:      []constraint.C{d},
				success: true,
				want: *segment.New(
					c.HP().L(),
					0,
					math.Inf(0),
				),
			}
		}(),

		// Check if we are overriding S.TMax() when appropriate.
		//
		// TMax is set if the direction of the new constraint C.D() lies
		// outside the existing constraint's feasiblity region F(D).
		func() config {
			p := *vector.New(1, 0)

			c := *constraint.New(
				*plane.New(
					p,
					*vector.New(1, 1),
				),
			)

			d := *constraint.New(
				*plane.New(
					p,
					*vector.New(1, 0),
				),
			)
			return config{
				name:    "SingleConstraint/Feasible/SetTMax",
				c:       c,
				cs:      []constraint.C{d},
				success: true,
				want: *segment.New(
					c.HP().L(),
					math.Inf(-1),
					0,
				),
			}
		}(),
	}

	testConfigs = append(
		testConfigs,

		// Assert that generateSegment is not invariant under order --
		// it is possible to fail with an infeasibility error in one
		// order, and return a valid segment if the order changes.
		//
		// It is the responsibility of the function calling
		// generateSegments to ensure order-invariance.
		//
		// Here, C and D form parallel lines, with both feasibility
		// regions pointing towards the positive X-axis and with the
		// constraint D lying further away from the origin; we will be
		// "relaxing" the constraint if C is added after D, which will
		// return an infeasibility error.
		func() []config {
			c := *constraint.New(
				*plane.New(
					*vector.New(1, 0),
					*vector.New(1, 0),
				),
			)

			d := *constraint.New(
				*plane.New(
					*vector.New(2, 0),
					*vector.New(1, 0),
				),
			)
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
					want: *segment.New(
						d.HP().L(),
						math.Inf(-1),
						math.Inf(0),
					),
				},
			}
		}()...)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := &R{
				constraints: c.cs,
			}
			got, ok := r.Add(c.c, tolerance)
			if ok != c.success {
				t.Errorf("generateSegment() = _, %v, want = _, %v", ok, c.success)
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					segment.S{},
					line.L{},
					vector.V{})); diff != "" {
				t.Errorf("generateSegment() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
