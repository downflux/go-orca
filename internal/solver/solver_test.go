package solver

import (
	"testing"

	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/solver/constraint"
)

const (
	tolerance = 1e-10
)

func TestOptimize(t *testing.T) {
	type config struct {
		name    string
		cs      []constraint.C
		v       vector.V
		success bool
		want    vector.V
	}

	testConfigs := []config{
		{
			name:    "NoConstraints",
			cs:      nil,
			v:       *vector.New(1, 2),
			success: true,
			want:    *vector.New(1, 2),
		},

		// The target minimization vector is already within the single
		// constraint.
		{
			name: "SingleConstraint/WithinConstraint",
			cs: []constraint.C{
				*constraint.New(
					*plane.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				),
			},
			v:       *vector.New(0, 2),
			success: true,
			want:    *vector.New(0, 2),
		},

		// The target minimization vector is outside the constraint, and
		// so must be recalculated.
		{
			name: "SingleConstraint/OutsideConstraint",
			cs: []constraint.C{
				*constraint.New(
					*plane.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				),
			},
			v:       *vector.New(0, -1),
			success: true,
			want:    *vector.New(0, 1),
		},
	}

	testConfigs = append(
		testConfigs,
		func() []config {
			c := *constraint.New(
				*plane.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			)

			d := *constraint.New(
				*plane.New(
					*vector.New(0, 2),
					*vector.New(0, 1),
				),
			)

			return []config{
				{
					name:    "ParalleConstraints/SuccessivelyConstrain",
					cs:      []constraint.C{c, d},
					v:       *vector.New(0, -1),
					success: true,
					want:    *vector.New(0, 2),
				},

				// Ensure that relaxing a parallel constraint
				// will not cause the function to throw an
				// infeasibility error.
				{
					name:    "ParalleConstraints/RelaxConstraintStillFeasible",
					cs:      []constraint.C{d, c},
					v:       *vector.New(0, -1),
					success: true,
					want:    *vector.New(0, 2),
				},
			}
		}()...,
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := optimize(c.v, c.cs, tolerance); ok != c.success || !vector.Within(c.want, got, tolerance) {
				t.Errorf("optimize() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
		})
	}
}
