package solver

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"

	v2d "github.com/downflux/go-geometry/2d/vector"
	s2d "github.com/downflux/go-orca/internal/solver/2d"
)

var _ s2d.O = func(s segment.S) vector.V { return project(s, v2d.V{}) }

func TestSolve(t *testing.T) {
	type config struct {
		name string
		cs   []constraint.C
		v    v2d.V
		r    float64
		want v2d.V
	}

	// Test 2D-feasible scenarios.
	testConfigs := []config{
		{
			name: "2D/NoConstraints",
			cs:   nil,
			v:    *v2d.New(1, 2),
			r:    math.Inf(0),
			want: *v2d.New(1, 2),
		},

		{
			name: "2D/TooFast",
			cs:   nil,
			v:    *v2d.New(0, 2),
			r:    1,
			want: *v2d.New(0, 1),
		},

		{
			name: "2D/OnlyRadialConstraint",
			cs:   nil,
			v:    *v2d.New(1, 2),
			r:    10,
			want: *v2d.New(1, 2),
		},

		// The target minimization vector is already within the single
		// constraint.
		{
			name: "2D/SingleConstraint/WithinConstraint",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			},
			v:    *v2d.New(0, 2),
			r:    math.Inf(0),
			want: *v2d.New(0, 2),
		},

		// The target minimization vector is outside the constraint, and
		// so must be recalculated.
		{
			name: "2D/SingleConstraint/OutsideConstraint",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			},
			v:    *v2d.New(0, -1),
			r:    math.Inf(0),
			want: *v2d.New(0, 1),
		},
	}

	testConfigs = append(
		testConfigs,
		func() []config {
			c := *constraint.New(
				*vector.New(0, 1),
				*vector.New(0, 1),
			)

			d := *constraint.New(
				*vector.New(0, 2),
				*vector.New(0, 1),
			)

			return []config{
				{
					name: "2D/ParalleConstraints/SuccessivelyConstrain",
					cs:   []constraint.C{c, d},
					v:    *v2d.New(0, -1),
					r:    math.Inf(0),
					want: *v2d.New(0, 2),
				},

				// Ensure that relaxing a parallel constraint
				// will not cause the function to throw an
				// infeasibility error.
				{
					name: "2D/ParalleConstraints/RelaxConstraintStillFeasible",
					cs:   []constraint.C{d, c},
					v:    *v2d.New(0, -1),
					r:    math.Inf(0),
					want: *v2d.New(0, 2),
				},
			}
		}()...,
	)

	testConfigs = append(
		testConfigs,
		func() []config {
			cs := []constraint.C{
				*constraint.New(
					*v2d.New(0, 1),
					*v2d.New(0, 1),
				),
				*constraint.New(
					*v2d.New(0, -1),
					*v2d.New(0, -1),
				),
			}

			return []config{
				{
					name: "3D/Trivial/2DInfeasible",
					cs:   cs,
					v:    *v2d.New(0, 1),
					want: *v2d.New(0, -10),
					r:    10,
				},
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := Solve(c.cs, c.v, c.r); !v2d.Within(c.want, got) {
				t.Errorf("Solve() = %v, want = %v", got, c.want)
			}
		})
	}
}
