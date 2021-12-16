package orca

import (
	"fmt"
	"math"
	"math/rand"
	"runtime"
	"testing"

	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/x/kd"
	"github.com/downflux/go-orca/agent"

	v2d "github.com/downflux/go-geometry/2d/vector"
	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

var _ agent.A = mock.A{}
var _ P[mock.A] = p{}

type p mock.A

func (q p) A() agent.A  { return mock.A(q) }
func (q p) P() vector.V { return vector.V(q.A().P()) }

func rn() float64 { return rand.Float64()*200 - 100 }
func rv() v2d.V   { return *v2d.New(rn(), rn()) }
func ra() mock.A {
	return *mock.New(
		mock.O{
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
func rt(n int) *kd.T[P[mock.A]] {
	// Generating large number of points in tests will mess with data
	// collection figures. We should ignore these allocs.
	runtime.MemProfileRate = 0
	defer func() { runtime.MemProfileRate = 512 * 1024 }()

	ps := make([]P[mock.A], 0, n)
	for i := 0; i < n; i++ {
		ps = append(ps, p(ra()))
	}
	t, _ := kd.New(ps)
	return t
}

func BenchmarkStep(b *testing.B) {
	type config struct {
		name string
		t    *kd.T[P[mock.A]]
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
		for _, p := range kd.Data[P[mock.A]](c.t) {
			s = math.Max(s, p.A().S())
		}

		b.Run(c.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				if _, err := Step[mock.A](O[mock.A]{
					T:        c.t,
					Tau:      1e-2,
					F:        func(a mock.A) bool { return true },
					PoolSize: c.size,
				}); err != nil {
					b.Errorf("Step() = _, %v, want = _, %v", err, nil)
				}
			}
		})
	}
}
