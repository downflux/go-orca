package lp

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

const tolerance = 1e-10

var (
	_ C = Reference{}
	_ C = plane.HP{}
)

type Reference struct {
	p vector.V
	n vector.V
}

func (r Reference) D() int { return 2 }
func (r Reference) A() []float64 {
	a := vector.Scale(-1, r.n)
	return []float64{a.X(), a.Y()}
}

func (r Reference) B() float64 {
	a := *vector.New(r.A()[0], r.A()[1])
	return vector.Dot(a, r.p)
}
func (r Reference) In(v vector.V) bool {
	a := *vector.New(r.A()[0], r.A()[1])
	return vector.Dot(a, v) <= r.B()
}

// rn returns a random int between [-100, 100).
func rn() float64 { return rand.Float64()*200 - 100 }

// ra returns a vector with randomized coordinates. The constructed vector must
// have a non-zero length.
func rv() vector.V {
	var v vector.V
	for {
		v = *vector.New(rn(), rn())
		if !vector.Within(v, vector.V{}, tolerance) {
			break
		}
	}
	return v
}

// within checks that two numeric values are within a small range of one
// another.
func within(got float64, want float64, tolerance float64) bool { return math.Abs(got-want) < tolerance }

func TestConformance(t *testing.T) {
	const nTests = 1000
	const delta = 1e-10

	type testConfig struct {
		name string
		p    vector.V
		n    vector.V
		vs   []vector.V
	}
	testConfigs := []testConfig{
		{
			name: "Horizontal",
			p:    *vector.New(0, 0),
			n:    *vector.New(1, 0),
			vs: []vector.V{
				*vector.New(0, 0),
				*vector.New(1, 0),
				*vector.New(-1, 0),
			},
		},
		{
			name: "Vertical",
			p:    *vector.New(0, 0),
			n:    *vector.New(0, 1),
			vs: []vector.V{
				*vector.New(0, 0),
				*vector.New(0, 1),
				*vector.New(0, -1),
			},
		},
		{
			name: "Sloped",
			p:    *vector.New(0, 0),
			n:    *vector.New(1, 1),
			vs: []vector.V{
				*vector.New(0, 0),
				*vector.New(1, 1),
				*vector.New(-1, -1),
			},
		},
		{
			name: "SlopedOffset",
			p:    *vector.New(0, 1),
			n:    *vector.New(1, 1),
			vs: []vector.V{
				*vector.New(0, 0),
				*vector.New(1, 1),
				*vector.New(2, 2),
			},
		},
	}

	for i := 0; i < nTests; i++ {
		c := testConfig{
			name: fmt.Sprintf("Random-%v", i),
			p:    rv(),
			n:    rv(),
		}
		for i := 0; i < nTests; i++ {
			c.vs = append(c.vs, rv())
		}
		testConfigs = append(testConfigs, c)
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := Reference{p: c.p, n: c.n}
			p := *plane.New(c.p, c.n)

			t.Run("In", func(t *testing.T) {
				for _, v := range c.vs {
					want := r.In(v)
					if got := p.In(v); got != want {
						t.Errorf("in() = %v, want = %v", got, want)
					}
				}
			})
			t.Run("A", func(t *testing.T) {
				want := *vector.New(r.A()[0], r.A()[1])
				got := *vector.New(p.A()[0], p.A()[1])
				if !vector.Within(want, got, tolerance) {
					t.Errorf("A() = %v, want = %v", got, want)
				}
			})
			t.Run("B", func(t *testing.T) {
				want := r.B()
				if got := p.B(); !within(want, got, tolerance) {
					t.Errorf("B() = %v, want = %v", got, want)
				}
			})
		})
	}
}
