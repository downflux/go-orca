package wall

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/external/snape/RVO2/vo/wall"
	"github.com/downflux/go-orca/vo"

	agentimpl "github.com/downflux/go-orca/internal/agent"
)

var (
	_ vo.VO = VO{}
)

func rn() float64  { return 200*rand.Float64() - 100 }
func rv() vector.V { return *vector.New(rn(), rn()) }
func ra() agent.A {
	return *agentimpl.New(
		agentimpl.O{
			P: rv(),
			V: rv(),
			R: rn() + 100,
		},
	)
}
func rs() segment.S { return *segment.New(*line.New(rv(), rv()), 0, 100) }

func TestConformance(t *testing.T) {
	const n = 1e4
	// minTau is the lower bound on tau values for agent-agent VOs. A tau
	// lower than this will cause an error.
	const minTau = 1e-3

	type config struct {
		name     string
		obstacle segment.S
		agent    agent.A
		tau      float64
	}

	testConfigs := []config{
		{
			name: "Manual/TestDomain/Region=3",
			obstacle: *segment.New(
				*line.New(
					/* p = */ *vector.New(-2, 2),
					/* d = */ *vector.New(1, 0),
				),
				0,
				4,
			),
			agent: *agentimpl.New(agentimpl.O{
				P: *vector.New(0, 0),
				V: *vector.New(0, 4),
				R: 1,
			}),
			tau: 1,
		},
	}

	for i := 0; i < n; i++ {
		testConfigs = append(testConfigs, config{
			name:     fmt.Sprintf("Random-%d", i),
			obstacle: rs(),
			agent:    ra(),
			tau:      math.Max(minTau, (rn() + 100)),
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			a := New(c.obstacle)
			b := wall.New(c.obstacle)
			t.Run(fmt.Sprintf("%v/ORCA/N", c.name), func(t *testing.T) {
				got := a.ORCA(c.agent, c.tau)[0].N()
				want := b.ORCA(c.agent, c.tau)[0].N()
				if !vector.WithinEpsilon(got, want, epsilon.Relative(0.05)) {
					t.Errorf("N() = %v, want = %v", got, want)
				}
			})
			t.Run(fmt.Sprintf("%v/ORCA/P", c.name), func(t *testing.T) {
				got := a.ORCA(c.agent, c.tau)[0]
				want := b.ORCA(c.agent, c.tau)[0]
				if !epsilon.Absolute(1e-5).Within(
					hyperplane.Line(
						got).Distance(want.P()),
					0,
				) {
					t.Errorf("P() = %v, want = %v", got.P(), want.P())
				}
			})
		})
	}
}
