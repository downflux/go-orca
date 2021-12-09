package orca

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	mock "github.com/downflux/go-orca/internal/agent/testdata/mock"
)

var _ point.P = P{}

func rn() float64  { return rand.Float64()*200 - 100 }
func rv() vector.V { return *vector.New(rn(), rn()) }
func ra() mock.A {
	return *mock.New(
		mock.O{
			P: rv(),
			V: vector.Scale(rand.Float64()*.5, vector.Unit(rv())),
			T: rv(),
		},
	)
}
func t(n int) *kd.T {
	ps := make([]point.P, 0, n)
	for i := 0; i < n; i++ {
		ps = append(ps, *New(ra()))
	}
	t, _ := kd.New(ps)
	return t
}

func BenchmarkStep(b *testing.B) {
	type config struct {
		name string
		t    *kd.T
	}

	testConfigs := []config{}
	for i := 0; i < 5; i++ {
		n := int(math.Pow(10, float64(i)))
		testConfigs = append(testConfigs, config{
			name: fmt.Sprintf("N=%v", n),
			t:    t(n),
		})
	}

	for _, c := range testConfigs {
		b.Run(c.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				if _, err := Step(c.t, 1e-2); err != nil {
					b.Errorf("Step() = _, %v, want = _, %v", err, nil)
				}
			}
		})
	}
}
