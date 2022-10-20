package orca

import (
	"fmt"
	"math"
	"math/rand"
	"runtime"
	"testing"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-orca/agent"
	"github.com/google/go-cmp/cmp"

	v2d "github.com/downflux/go-geometry/2d/vector"
	agentimpl "github.com/downflux/go-orca/internal/agent"
)

var (
	_ P = p{}
)

type p struct {
	a agent.A
}

func (p p) A() agent.A  { return p.a }
func (p p) P() vector.V { return vector.V(p.a.P()) }

func rn() float64 { return rand.Float64()*200 - 100 }
func rv() v2d.V   { return *v2d.New(rn(), rn()) }
func ra() agentimpl.A {
	return *agentimpl.New(
		agentimpl.O{
			P: rv(),
			V: v2d.Scale(rand.Float64()*.5, v2d.Unit(rv())),
			R: rand.Float64(),
			T: rv(),
			// Ensure the agent's target vector is inside the
			// bounding circle.
			S: rn() + 100,
		},
	)
}
func rt(n int) *kd.KD[P] {
	// Generating large number of points in tests will mess with data
	// collection figures. We should ignore these allocs.
	runtime.MemProfileRate = 0
	defer func() { runtime.MemProfileRate = 512 * 1024 }()

	ps := make([]P, 0, n)
	for i := 0; i < n; i++ {
		a := ra()
		ps = append(ps, p{a: &a})
	}
	t := kd.New(kd.O[P]{
		Data: ps,
		K:    2,
		N:    1,
	})
	return t
}

func TestStep(t *testing.T) {
	type config struct {
		name   string
		agents []agent.A
		tau    float64
		f      func(a agent.A) bool

		want []Mutation
	}

	testConfigs := []config{
		func() config {
			a := agentimpl.New(
				agentimpl.O{
					P: *v2d.New(1, 2),
					V: *v2d.New(2, 3),
					T: *v2d.New(3, 4),
					S: 10,
				},
			)

			return config{
				name:   "PointerEqualityComparison",
				agents: []agent.A{a},
				tau:    1e-2,
				f:      func(agent.A) bool { return true },
				want: []Mutation{
					Mutation{
						A: a,
						V: a.T(),
					},
				},
			}
		}(),
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			var ps []P
			for _, a := range c.agents {
				ps = append(ps, p{a: a})
			}

			tr := kd.New(kd.O[P]{
				Data: ps,
				K:    2,
				N:    1,
			})

			s := math.Inf(-1)
			for _, p := range kd.Data(tr) {
				s = math.Max(s, p.A().S())
			}

			got, err := Step(O[P]{
				T:        tr,
				Tau:      c.tau,
				F:        c.f,
				PoolSize: 1,
			})
			if err != nil {
				t.Errorf("Step() = _, %v, want = _, %v", got, nil)
			}
			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					agentimpl.A{},
					hypersphere.C{},
				),
			); diff != "" {
				t.Errorf("Step() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func BenchmarkStep(b *testing.B) {
	type config struct {
		name string
		t    *kd.KD[P]
		size int
	}

	testConfigs := []config{}
	for n := 1000; n < 1000000; n = n * 10 {
		for size := 1; size <= n && size < 128; size = size << 1 {
			testConfigs = append(testConfigs, config{
				name: fmt.Sprintf("PoolSize=%v/N=%v", size, n),
				t:    rt(n),
				size: size,
			})
		}
	}

	for _, c := range testConfigs {
		s := math.Inf(-1)
		for _, p := range kd.Data(c.t) {
			s = math.Max(s, p.A().S())
		}

		b.Run(c.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				if _, err := Step(O[P]{
					T:        c.t,
					Tau:      1e-2,
					F:        func(a agent.A) bool { return true },
					PoolSize: c.size,
				}); err != nil {
					b.Errorf("Step() = _, %v, want = _, %v", err, nil)
				}
			}
		})
	}
}
