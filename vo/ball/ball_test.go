package ball

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/orca/vector"
	"github.com/downflux/orca/vo"
)

var (
	_ vo.Agent = Agent{}
	_ vo.VO    = VO{}
	_ vo.VO    = Reference{}
)

type Agent struct {
	p vector.V
	v vector.V
	r float64
}

func (a Agent) P() vector.V { return a.p }
func (a Agent) V() vector.V { return a.v }
func (a Agent) R() float64  { return a.r }

// Reference implements the official RVO2 spec. See
// https://gamma.cs.unc.edu/RVO2/ for more information.
type Reference struct {
	a vo.Agent
	b vo.Agent
}

func (v Reference) ORCA() vector.V { return vector.V{} }
func (v Reference) r() float64     { return v.a.R() + v.b.R() }
func (v Reference) p() vector.V    { return vector.Sub(v.b.P(), v.a.P()) }
func (v Reference) w() vector.V    { return vector.Sub(vector.Sub(v.a.V(), v.b.V()), v.p()) }
func (v Reference) check() Direction {
	if vector.SquaredMagnitude(v.p()) <= math.Pow(v.r(), 2) {
		return Collision
	}

	wp := vector.Dot(v.w(), v.p())
	if wp < 0 && math.Pow(wp, 2) > vector.SquaredMagnitude(v.w())*math.Pow(v.r(), 2) {
		return Circle
	}

	if vector.Determinant(v.p(), v.w()) > 0 {
		return Left
	}

	return Right
}

// TestVOReferenceDirection asserts a simple agent-agent setup will return u in
// the correct hand-calculated direction.
func TestVOReferenceDirection(t *testing.T) {
	a := Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1}
	b := Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2}
	want := Direction(Circle)

	if got := (Reference{a: a, b: b}).check(); got != want {
		t.Errorf("check() = %v, want = %v", got, want)
	}
}

// r returns a random int between [-100, 100).
func r() float64 { return rand.Float64()*200 - 100 }

// TestVODirectionConformance tests that agent-agent VOs will return u in the
// correct direction using the reference implementation as a sanity check.
func TestVODirectionConformance(t *testing.T) {
	const nTests = 1000
	const tolerance = 1e-10

	type testConfig struct {
		name string
		a    vo.Agent
		b    vo.Agent
	}

	testConfigs := []testConfig{
		{
			name: "TestSimpleCase",
			a:    Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1},
			b:    Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2},
		},
		{
			name: "TestCollision",
			a:    Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1},
			b:    Agent{p: *vector.New(0, 3), v: *vector.New(1, -1), r: 2},
		},
	}

	for i := 0; i < nTests; i++ {
		testConfigs = append(testConfigs, testConfig{
			name: fmt.Sprintf("TestRandom-%v", i),
			a:    Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
			b:    Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			want := (Reference{a: c.a, b: c.b}).check()

			v := *New(c.a, c.b)
			if got := v.check(); got != want {
				beta, _ := v.beta()
				theta, _ := v.theta()

				// Disregard rounding errors around where 𝜃 ~ 𝛽.
				if got == Circle && (math.Abs(beta-theta) > tolerance || math.Abs(2*math.Pi-theta-beta) > tolerance) {
					return
				}
				t.Errorf("check() = %v, want = %v", got, want)
			}
		})
	}
}

func BenchmarkVOReference(b *testing.B) {
	for i := 0; i < b.N; i++ {
		(Reference{
			a: Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
			b: Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
		}).check()
	}
}

func BenchmarkVO(b *testing.B) {
	for i := 0; i < b.N; i++ {
		(*New(
			Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
			Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
		)).check()
	}
}
