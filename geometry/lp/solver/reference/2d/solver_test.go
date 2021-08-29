package solver

import (
	"testing"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/geometry/plane"

	agent "github.com/downflux/orca/agent/reference"
)

const (
	tolerance = 1e-10
)

var (
	_ helper.Agent = maxSpeedAgent{}
)

func TestMaxSpeedAgent(t *testing.T) {
	testConfigs := []struct{
		name string
		s float64
		t vector.V
		want vector.V
	}{
		{name: "SmallS/NoScale", s: 1, t: *vector.New(0, 0.5), want: *vector.New(0, 0.5)},
		{name: "SmallS/Scale", s: 1, t: *vector.New(0, 1.5), want: *vector.New(0, 1)},
		{name: "SmallS/Boundary", s: 1, t: *vector.New(0, 1), want: *vector.New(0, 1)},
		{name: "LargeS/NoScale", s: 7, t: *vector.New(0, 1), want: *vector.New(0, 1)},
		{name: "LargeS/Scale", s: 7, t: *vector.New(0, 10), want: *vector.New(0, 7)},
		{name: "LargeS/Boundary", s: 7, t: *vector.New(0, 7), want: *vector.New(0, 7)},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			a := maxSpeedAgent{*agent.New(agent.O{T: c.t, S: c.s})}

			if got := a.T(); !vector.Within(got, c.want, tolerance) {
				t.Errorf("T() = %v, want = %v", got, c.want)
			}
		})
	}
}

// TestSimpleRefinedConstraintNoImproveSolver checks the solver behaves as
// expected when a later (valid) constraint is superseded by an earlier
// constraint -- while helper.H will fail in this individual case, the 2D solver
// should handle this outside the helper and resolve as expected -- that is,
// ignore the later constraint all together.
func TestSimpleRefinedConstraintNoImproveSolver(t *testing.T) {
	want := *vector.New(0, 0.5)

	got, ok := S{}.Solve(
		*agent.New(agent.O{T: *vector.New(0, 1), S: 5}),
		[]plane.HP{
			*plane.New(*vector.New(0, 0.5), *vector.New(0, -1)),

			// Introduce a plane "further away" from the preferred
			// velocity in velocity-space, with valid velocities in
			// this half plane being more subotimal than the first
			// constraint.
			*plane.New(*vector.New(0, -0.5), *vector.New(0, -1)),
		},
	)
	if !ok || !vector.Within(got, want, tolerance) {
		t.Errorf("Solve() = %v, %v, want = %v, true", got, ok, want)
	}
}
