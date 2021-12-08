package orca

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
)

var _ point.P = P{}
var _ agent.RW = &mockAgent{}

func rn() float64  { return rand.Float64()*200 - 100 }
func rv() vector.V { return *vector.New(rn(), rn()) }
func ra() mockAgent {
	return mockAgent{
		p: rv(),
		v: vector.Scale(rand.Float64()*.5, vector.Unit(rv())),
		t: rv(),
	}
}

// TODO(minkezhang): Use mock agent instead.
type mockAgent struct {
	p vector.V
	v vector.V
	t vector.V
}

func (a *mockAgent) P() vector.V     { return a.p }
func (a *mockAgent) V() vector.V     { return a.v }
func (a *mockAgent) R() float64      { return 1 }
func (a *mockAgent) T() vector.V     { return a.t }
func (a *mockAgent) S() float64      { return .5 }
func (a *mockAgent) SetV(v vector.V) { a.v = v }

func BenchmarkStep(b *testing.B) {
	type config struct {
		name string
		n    int
	}

	testConfigs := []config{}
	for i := 0; i < 5; i++ {
		n := int(math.Pow(10, float64(i)))
		testConfigs = append(testConfigs, config{
			name: fmt.Sprintf("N=%v", n),
			n:    n,
		})
	}

	for _, c := range testConfigs {
		b.Run(c.name, func(b *testing.B) {
			agents := make([]agent.RW, 0, c.n)
			for i := 0; i < c.n; i++ {
				a := ra()
				agents = append(agents, &a)
			}
			o := New(agents)

			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				if err := o.Step(1e-2); err != nil {
					b.Errorf("Step() = %v, want = %v", err, nil)
				}
			}
		})
	}
}
