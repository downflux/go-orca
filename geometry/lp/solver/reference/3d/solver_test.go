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
	}

	testConfigs = append(testConfigs,
		config{
			name: "SingleConstraint/NoOpt",
			a:    A{r: 1, t: *vector.New(0, 0.6)},
			cs: []plane.HP{
				*plane.New(*vector.New(0, 0.5), *vector.New(0, -1)),
			},
			success: true,
			want:    *vector.New(0, 0.6),
		},
		config{
			name: "SingleConstraint/Opt",
			a:    A{r: 1, t: *vector.New(0, 0.6)},
			cs: []plane.HP{
				*plane.New(*vector.New(0, 0.5), *vector.New(0, 1)),
			},
			success: true,
			want:    *vector.New(0, 1),
		},
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			s := S{}
			got, ok := s.Solve(c.a, c.cs)
			if ok != c.success || !vector.Within(got, c.want, tolerance) {
				t.Errorf("Solve() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
			for _, p := range c.cs {
				// TODO(minkezhang): Fix this.
				if ok := p.In(got); !ok {
					t.Errorf("%v.In(%v) = %v, want = %v; N() == %v, D() == %v", p, got, ok, true, p.N(), p.D())
				}
			}
		})
	}
}
