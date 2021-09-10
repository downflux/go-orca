package solver

import (
	"testing"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"

	agent "github.com/downflux/orca/agent/reference"
)

const (
	tolerance = 1e-10
)

var (
	_ helper.Agent = maxSpeedAgent{}
)

func TestMaxSpeedAgent(t *testing.T) {
	testConfigs := []struct {
		name string
		s    float64
		t    vector.V
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
			a := maxSpeedAgent{
				Agent: *agent.New(agent.O{T: c.t, S: c.s}),
			}

			if got := a.T(); !vector.Within(got, c.want, tolerance) {
				t.Errorf("T() = %v, want = %v", got, c.want)
			}
		})
	}
}

// TestSimpleRefinedConstraintNoImproveSolver checks the solver ignores a later
// (valid) constraint when it is superseded by an existing solution.
func TestSimpleRefinedConstraintNoImproveSolver(t *testing.T) {
	want := *vector.New(0, -0.5)

	got, ok := S{}.Solve(
		*agent.New(agent.O{T: *vector.New(0, 1), S: 5}),
		[]plane.HP{
			*plane.New(*vector.New(0, -0.5), *vector.New(0, -1)),

			// The result obtained from the previous constraint is a
			// point on the constraint line (0, -0.5); this is the
			// prescribed velocity for the agent to travel. Because
			// the new constraint fully covers the existing
			// constraint, any solution to the first solution is a
			// solution to the second one as well.
			//
			// N.B.: Note that order here matters; if we were to
			// solve for the second constraint first and then solve
			// for the first, we will in fact get an infeasibility
			// error.
			//
			// TODO(minkezhang): Verify this is intended behavior of
			// the RVO2 implementation.
			*plane.New(*vector.New(0, 0.5), *vector.New(0, -1)),
		},
	)
	if !ok || !vector.Within(got, want, tolerance) {
		t.Errorf("Solve() = %v, %v, want = %v, true", got, ok, want)
	}
}
