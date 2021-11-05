package solver

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/solver/internal/helper"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
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
		opt  bool
		want vector.V
	}{
		{name: "SmallS/NoScale", s: 1, t: *vector.New(0, 0.5), want: *vector.New(0, 0.5)},
		{name: "SmallS/Scale", s: 1, t: *vector.New(0, 1.5), want: *vector.New(0, 1)},
		{name: "SmallS/Boundary", s: 1, t: *vector.New(0, 1), want: *vector.New(0, 1)},
		{name: "LargeS/NoScale", s: 7, t: *vector.New(0, 1), want: *vector.New(0, 1)},
		{name: "LargeS/Scale", s: 7, t: *vector.New(0, 10), want: *vector.New(0, 7)},
		{name: "LargeS/Boundary", s: 7, t: *vector.New(0, 7), want: *vector.New(0, 7)},

		{name: "KnownOptimalVelocity", s: 1, t: *vector.New(0, 0.5), opt: true, want: *vector.New(0, 1)},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			a := maxSpeedAgent{
				Agent:                 *mock.New(mock.O{T: c.t, S: c.s}),
				knownOptimalMagnitude: c.opt,
			}

			if got := a.T(); !vector.Within(got, c.want, tolerance) {
				t.Errorf("T() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestSolve(t *testing.T) {
	testConfigs := []struct {
		name string
		cs   []plane.HP
		a    mock.A
		opt  bool
		want vector.V
	}{
		{
			// SimpleRefinedConstraintNoImproveSolver checks the
			// solver ignores a later (valid) constraint when it is
			// superseded by an existing solution.
			name: "SimpleRefinedConstraintNoImproveSolver",
			a:    *mock.New(mock.O{T: *vector.New(0, 1), S: 5}),
			cs: []plane.HP{
				*plane.New(*vector.New(0, -0.5), *vector.New(0, -1)),

				// The result obtained from the previous
				// constraint is a point on the constraint line
				// (0, -0.5); this is the prescribed velocity
				// for the agent to travel. Because the new
				// constraint fully covers the existing
				// constraint, any solution to the first
				// solution is a solution to the second one as
				// well.
				//
				// N.B.: Note that order here matters; if we
				// were to solve for the second constraint first
				// and then solve for the first, we will in fact
				// get an infeasibility error.
				//
				// TODO(minkezhang): Verify this is intended
				// behavior of the RVO2 implementation.
				*plane.New(*vector.New(0, 0.5), *vector.New(0, -1)),
			},
			want: *vector.New(0, -0.5),
		},
		{
			name: "SimpleDirOptSolver",
			a:    *mock.New(mock.O{T: *vector.New(0, 1), S: 5}),
			cs: []plane.HP{
				// D() points in the positive X direction.
				*plane.New(*vector.New(0, -0.5), *vector.New(0, -1)),
			},
			opt:  true,
			want: *vector.New(math.Sqrt(25-0.25), -0.5),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			got, ok := S{knownOptimalMagnitude: c.opt}.Solve(c.a, c.cs)
			if !ok || !vector.Within(got, c.want, tolerance) {
				t.Errorf("Solve() = %v, %v, want = %v, true", got, ok, c.want)
			}
		})
	}
}
