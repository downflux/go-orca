package ball

import (
	"fmt"
	"math"
	"math/rand"
	"strings"
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

func random() float64 { return math.Floor(rand.Float64()*200 - 100) }

// TestVODirectionConformance tests that agent-agent VOs will return u in the
// correct direction using the reference implementation as a sanity check.
func TestVODirectionConformance(t *testing.T) {
	nTests := 100

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
			a:    Agent{p: *vector.New(random(), random()), v: *vector.New(random(), random()), r: math.Abs(random())},
			b:    Agent{p: *vector.New(random(), random()), v: *vector.New(random(), random()), r: math.Abs(random())},
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			want := (Reference{a: c.a, b: c.b}).check()
			v := *New(c.a, c.b)
			if got := v.check(); got != want {
				beta, betaErr := v.beta()
				theta, thetaErr := v.theta()
				t.Error(strings.Join([]string{
					fmt.Sprintf("check() = %v, want = %v", got, want),
					fmt.Sprintf("\ta.R() = %v", c.a.R()),
					fmt.Sprintf("\ta.P() = %v", c.a.P()),
					fmt.Sprintf("\ta.V() = %v", c.a.V()),
					fmt.Sprintf("\tb.R() = %v", c.b.R()),
					fmt.Sprintf("\tb.P() = %v", c.b.P()),
					fmt.Sprintf("\tb.V() = %v", c.b.V()),
					fmt.Sprintf("\tvo.r() = %v", v.r()),
					fmt.Sprintf("\tvo.p() = %v", v.p()),
					fmt.Sprintf("\tvo.w() = %v", v.w()),
					fmt.Sprintf("\tvo.beta() = %v (%v°), %v", beta, beta*180/math.Pi, betaErr),
					fmt.Sprintf("\tvo.theta() = %v (%v°), %v", theta, theta*180/math.Pi, thetaErr),
				}, "\n"))
			}
		})
	}
}
