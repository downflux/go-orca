package constraint

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"

	mock "github.com/downflux/go-orca/internal/solver/constraint/testdata/mock"
)

const (
	tolerance = 1e-10
)

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
			name: "Vertical",
			p:    *vector.New(0, 0),
			n:    *vector.New(1, 0),
			vs: []vector.V{
				*vector.New(0, 0),
				*vector.New(1, 0),
				*vector.New(-1, 0),
			},
		},
		{
			name: "Horizontal",
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
			p := *New(*plane.New(c.p, c.n))
			s := *mock.New(p.A(), p.B())

			t.Run("In", func(t *testing.T) {
				for _, v := range c.vs {
					want := p.In(v)
					if got := s.In(v); got != want {
						t.Errorf("In() = %v, want = %v", got, want)
					}
				}
			})
		})
	}
}
