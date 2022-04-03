package cone

import (
	"fmt"
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestL(t *testing.T) {
	type config struct {
		name   string
		center hypersphere.C
		l      vector.V
		r      vector.V
	}
	testConfigs := []config{
		{
			name:   "Trivial",
			center: *hypersphere.New(*vector.New(0, 2), 1),
			l:      *vector.New(-math.Sqrt(3)/2, 1.5),
			r:      *vector.New(-math.Sqrt(3)/2, -1.5),
		},
		{
			name:   "345",
			center: *hypersphere.New(*vector.New(0, 5), 3),
			l:      *vector.New(-2.4, 3.2),
			r:      *vector.New(-2.4, -3.2),
		},
		// Check that L and R are calculated with respect to the view of
		// the agent.
		{
			name:   "345/Mirror",
			center: *hypersphere.New(*vector.New(0, -5), 3),
			l:      *vector.New(2.4, -3.2),
			r:      *vector.New(2.4, 3.2),
		},
		// 𝜏 values in the ORCA context is a time scalar factor attached
		// to the truncated velocity cone. Larger 𝜏 values indicate the
		// cone needs to exclude more velocity values, i.e. the base of
		// the truncated cone will approach the origin and shrink
		// accordingly.
		func() config {
			tau := 101.0
			return config{
				name: "345LargeTau",
				center: *hypersphere.New(
					vector.Scale(
						1.0/tau,
						*vector.New(0, 5),
					),
					3.0/tau,
				),
				l: vector.Scale(1.0/tau, *vector.New(-2.4, 3.2)),
				r: vector.Scale(1.0/tau, *vector.New(-2.4, -3.2)),
			}
		}(),
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			co, err := New(c.center)
			if err != nil {
				t.Fatalf("New() unexpectedly failed: %v", err)
			}
			t.Run(fmt.Sprintf("%s/L", c.name), func(t *testing.T) {
				if got := co.L(); !vector.Within(got, c.l) {
					t.Errorf("L() = %v, want = %v", got, c.l)
				}
			})
			t.Run(fmt.Sprintf("%s/R", c.name), func(t *testing.T) {
				if got := co.R(); !vector.Within(got, c.r) {
					t.Errorf("R() = %v, want = %v", got, c.r)
				}
			})
		})
	}
}
