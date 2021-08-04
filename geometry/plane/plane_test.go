package plane

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/orca/geometry/vector"
)

const tolerance = 1e-10

var (
	_ LC = Reference{}
	_ LC = HP{}
)

type Reference struct {
	p vector.V
	n vector.V
}

func (r Reference) A() vector.V { return vector.Scale(-1, r.n) }
func (r Reference) B() float64  { return vector.Dot(r.A(), r.p) }
func (r Reference) in(v vector.V) bool {
	return vector.Dot(r.A(), v) <= r.B()
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
			p := *New(c.p, c.n)

			t.Run("In", func(t *testing.T) {
				for _, v := range c.vs {
					want := r.in(v)
					if got := p.in(v); got != want {
						t.Errorf("in() = %v, want = %v", got, want)
					}
				}
			})
			t.Run("A", func(t *testing.T) {
				want := r.A()
				if got := p.A(); !vector.Within(want, got, tolerance) {
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

func TestIn(t *testing.T) {
	type check struct {
		v    vector.V
		want bool
	}
	testConfigs := []struct {
		name  string
		hp    HP
		l     vector.V
		tests []check
	}{
		{
			name: "Horizontal",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(1, 0),
			),
			l: *vector.New(0, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(1, 0), want: true},
				{v: *vector.New(-1, 0), want: false},
			},
		},
		{
			name: "Vertical",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(0, 1),
			),
			l: *vector.New(-1, 0),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(0, 1), want: true},
				{v: *vector.New(0, -1), want: false},
			},
		},
		{
			name: "Sloped",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(1, 1),
			),
			l: *vector.New(-1, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(1, 1), want: true},
				{v: *vector.New(-1, -1), want: false},
			},
		},
		{
			name: "SlopedOffset",
			hp: *New(
				*vector.New(0, 1),
				*vector.New(1, 1),
			),
			l: *vector.New(-1, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: false},
				{v: *vector.New(1, 1), want: true},
				{v: *vector.New(2, 2), want: true},
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			for _, test := range c.tests {
				if got := c.hp.l(); !vector.Within(got, c.l, tolerance) {
					t.Fatalf("l() = %v, want = %v", got, c.l)
				}
				if got := c.hp.in(test.v); got != test.want {
					t.Errorf("in(%v) = %v, want = %v", test.v, got, test.want)
				}
			}
		})
	}
}
