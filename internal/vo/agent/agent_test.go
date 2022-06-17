package agent

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/internal/agent"
	"github.com/downflux/go-orca/internal/vo/agent/opt"
	"github.com/downflux/go-orca/vo"

	mock "github.com/downflux/go-orca/external/snape/RVO2/vo/agent"
)

var (
	_ vo.VO = VO{}
)

// rn returns a random int between [-100, 100).
func rn() float64 { return rand.Float64()*200 - 100 }

// ra returns an agent with randomized dimensions.
func ra() agent.A {
	return *agent.New(
		agent.O{
			P: *vector.New(rn(), rn()),
			V: *vector.New(rn(), rn()),
			R: math.Abs(rn()),
		},
	)
}

// TestVOConformance tests that agent-agent VOs will return u in the correct
// domain using the mock implementation as a sanity check.
func TestVOConformance(t *testing.T) {
	const nTests = 1000
	const delta = 1e-10

	type config struct {
		name     string
		agent    agent.A
		obstacle agent.A
		tau      float64
	}

	testConfigs := []config{
		{
			name:     "SimpleCase",
			agent:    *agent.New(agent.O{P: *vector.New(0, 0), V: *vector.New(0, 0), R: 1}),
			obstacle: *agent.New(agent.O{P: *vector.New(0, 5), V: *vector.New(1, -1), R: 2}),
			tau:      1,
		},
		{
			name:     "SimpleCaseLargeTimeStep",
			agent:    *agent.New(agent.O{P: *vector.New(0, 0), V: *vector.New(0, 0), R: 1}),
			obstacle: *agent.New(agent.O{P: *vector.New(0, 5), V: *vector.New(1, -1), R: 2}),
			tau:      3,
		},
		{
			name:     "Collision",
			agent:    *agent.New(agent.O{P: *vector.New(0, 0), V: *vector.New(0, 0), R: 1}),
			obstacle: *agent.New(agent.O{P: *vector.New(0, 3), V: *vector.New(1, -1), R: 2}),
			tau:      1,
		},
	}

	for i := 0; i < nTests; i++ {
		testConfigs = append(testConfigs, config{
			name:     fmt.Sprintf("Random-%v", i),
			agent:    ra(),
			obstacle: ra(),
			// A simulation timestep scalar of 0 indicates the
			// simulation will never advance to the next snapshot,
			// which is a meaningless case (and will produce
			// boundary condition errors in our implementation).
			tau: math.Abs(rn()) + delta,
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			v := New(
				c.obstacle,
				opt.O{
					Weight: opt.WeightEqual,
					VOpt:   opt.VOptV,
				},
			)
			t.Run("ORCA", func(t *testing.T) {
				want := mock.New(c.obstacle).ORCA(c.agent, float64(c.tau))
				got := v.ORCA(c.agent, c.tau)

				if !hyperplane.WithinEpsilon(got, want, epsilon.Absolute(1e-5)) {
					t.Errorf("ORCA() = %v, want = %v", got, want)
				}
			})
		})
	}
}

// BenchmarkORCA compares the relative performance of VO.domain() bewteen the
// official RVO2 spec vs. the custom implementation provided.
func BenchmarkORCA(t *testing.B) {
	testConfigs := []struct {
		name        string
		agent       agent.A
		constructor func(obstacle agent.A) vo.VO
	}{
		{
			name:        "VOReference",
			constructor: func(obstacle agent.A) vo.VO { return mock.New(obstacle) },
		},
		{
			name: "VO",
			constructor: func(obstacle agent.A) vo.VO {
				return New(
					obstacle,
					opt.O{
						Weight: opt.WeightEqual,
						VOpt:   opt.VOptV,
					},
				)
			},
		},
	}
	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.B) {
			a := ra()
			o := ra()
			v := c.constructor(o)

			t.ResetTimer()
			for i := 0; i < t.N; i++ {
				v.ORCA(a, 1)
			}
		})
	}
}
