package solver

import (
	"testing"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

var (
	_ helper.Agent = A{}
)

func TestSolve(t *testing.T) {
	type config struct {
		name    string
		a       A
		cs      []plane.HP
		success bool
		want    vector.V
	}

	testConfigs := []config{
		// If there is no constraint, the input from the seed condition
		// should be returned. Note that this remains the case even if
		// the seed condition velocity vector exceeds the maximum speed
		// (the circular constraint); this is to match existing RVO2
		// behavior.
		{
			name:    "NoConstraint",
			a:       A{r: 1, t: *vector.New(0, 1)},
			cs:      nil,
			success: true,
			want:    *vector.New(0, 1),
		},

		// Find a suitable solution when given solution is infeasible.
		{
			name: "SingleConstraint/InfeasibleOpt",
			a:    A{r: 1, t: *vector.New(0, 0.6)},
			cs: []plane.HP{
				*plane.New(*vector.New(0, 0.5), *vector.New(0, -1)),
			},
			success: true,
			want:    *vector.New(0, -1),
		},

		// If a solution is already feasible, do not optimize it.
		{
			name: "SingleConstraint/FeasibleNoOpt",
			a:    A{r: 1, t: *vector.New(0, 0.6)},
			cs: []plane.HP{
				*plane.New(*vector.New(0, 0.5), *vector.New(0, 1)),
			},
			success: true,
			want:    *vector.New(0, 0.6),
		},

		// In the case of constraints which have no common intersection,
		// the solver should return a target velocity vector which is
		// pointed in a direction that is minimally intrusive into the
		// forbidden regions.
		{
			name: "MultipleConstraint/MutuallyExclusive",
			a:    A{r: 1, t: *vector.New(0, 0)},
			cs: []plane.HP{
				*plane.New(*vector.New(0, 0.5), *vector.New(0, 1)),
				*plane.New(*vector.New(0, -0.5), *vector.New(0, -1)),
			},
			success: true,
			want:    *vector.New(-1, 0),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			s := S{}
			got, ok := s.Solve(c.a, c.cs)
			if ok != c.success || !vector.Within(got, c.want, tolerance) {
				t.Errorf("Solve() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
		})
	}
}
