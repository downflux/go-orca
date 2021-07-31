package ball

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/downflux/orca/vector"
	"github.com/downflux/orca/vo"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

var (
	_ vo.Agent = Agent{}
	_ vo.VO    = &VO{}
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
	a   vo.Agent
	b   vo.Agent
	tau float64
}

func (v Reference) ORCA() (vector.V, error) {
	switch d := v.check(); d {
	case Circle:
		return vector.Scale(v.r() / v.tau - vector.Magnitude(v.w()), v.w()), nil
	case Collision:
		return vector.Scale(v.r() * minTauScalar - vector.Magnitude(v.w()), v.w()), nil
	case Left:
		return vector.Sub(vector.Scale(vector.Dot(v.v(), v.l()), v.l()), v.v()), nil
	case Right:
		l := *vector.New(-v.l().X(), v.l().Y())
		return vector.Sub(vector.Scale(vector.Dot(v.v(), l), l), v.v()), nil
	}
	return vector.V{}, status.Error(codes.Unimplemented, "unimplemented function")
}
func (v Reference) r() float64  { return (v.a.R() + v.b.R()) / v.tau }
func (v Reference) p() vector.V { return vector.Scale(1/v.tau, vector.Sub(v.b.P(), v.a.P())) }
func (v Reference) w() vector.V { return vector.Sub(v.v(), v.p()) }
func (v Reference) v() vector.V { return vector.Sub(v.a.V(), v.b.V()) }
func (v Reference) l() vector.V {
	l := math.Sqrt(vector.SquaredMagnitude(v.p()) - math.Pow(v.r(), 2))
	return *vector.New(
		v.p().X() * l - v.p().Y() * v.r(),
		v.p().X() * v.r() + v.p().Y() * l,
	)
}

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

	testConfigs := []struct {
		name string
		tau  float64
		want Direction
	}{
		{name: "NormalScale", tau: 1, want: Circle},
		{name: "NormalScaleLargeTau", tau: 3, want: Left},
	}
	for _, c := range testConfigs {
		if got := (Reference{a: a, b: b, tau: c.tau}).check(); got != c.want {
			t.Errorf("check() = %v, want = %v", got, c.want)
		}
	}
}

func TestVOReferenceORCA(t *testing.T) {
	a := Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1}
	b := Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2}

	testConfigs := []struct {
		name string
		tau  float64
		want vector.V
	}{
		{name: "NormalScale", tau: 1},
		{name: "NormalScaleLargeTau", tau: 3},
	}
	for _, c := range testConfigs {
		v, _ := New(a, b, c.tau)
		fmt.Println(v.ORCA())
		if got, err := (Reference{a: a, b: b, tau: c.tau}).ORCA(); err != nil || got != c.want {
			t.Errorf("check() = %v, %v, want = %v, %v", got, err, c.want, nil)
		}
	}
}

// r returns a random int between [-100, 100).
func r() float64 { return rand.Float64()*200 - 100 }

// within checks that two numeric values are within a small range of one
// another.
func within(got float64, want float64, tolerance float64) bool {
	return math.Abs(got - want) < tolerance
}

// TestVODirectionConformance tests that agent-agent VOs will return u in the
// correct direction using the reference implementation as a sanity check.
func TestVODirectionConformance(t *testing.T) {
	const nTests = 1000
	const tolerance = 1e-10

	type testConfig struct {
		name string
		a    vo.Agent
		b    vo.Agent
		tau  float64
	}

	testConfigs := []testConfig{
		{
			name: "TestSimpleCase",
			a:    Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1},
			b:    Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2},
			tau:  1,
		},
		{
			name: "TestSimpleCaseLargeTimeStep",
			a:    Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1},
			b:    Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2},
			tau:  3,
		},
		{
			name: "TestCollision",
			a:    Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1},
			b:    Agent{p: *vector.New(0, 3), v: *vector.New(1, -1), r: 2},
			tau:  1,
		},
	}

	for i := 0; i < nTests; i++ {
		testConfigs = append(testConfigs, testConfig{
			name: fmt.Sprintf("TestRandom-%v", i),
			a:    Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
			b:    Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())},
			tau:  math.Abs(r()) + tolerance,
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			want := (Reference{a: c.a, b: c.b, tau: float64(c.tau)}).check()

			v, err := New(c.a, c.b, c.tau)
			if err != nil {
				t.Fatalf("New() returned a non-nil error: %v", err)
			}
			if got := v.check(); got != want {
				beta, _ := v.beta()
				theta, _ := v.theta()

				// Disregard rounding errors around where 𝜃 ~ 𝛽.
				if got == Circle && (!within(beta, theta, tolerance) || !within(2 * math.Pi - theta, beta, tolerance)) {
					return
				}
				t.Errorf("check() = %v, want = %v", got, want)
			}
		})
	}
}

// BenchmarkCheck compares the relative performance of VO.check() bewteen the
// official RVO2 spec vs. the custom implementation provided.
func BenchmarkCheck(t *testing.B) {
	type checker interface {
		check() Direction
	}

	testConfigs := []struct {
		name        string
		constructor func(a, b vo.Agent) checker
	}{
		{
			name:        "VOReference",
			constructor: func(a, b vo.Agent) checker { return Reference{a: a, b: b, tau: 1} },
		},
		{
			name: "VO",
			constructor: func(a, b vo.Agent) checker {
				v, _ := New(a, b, 1)
				return v
			},
		},
	}
	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.B) {
			a := Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())}
			b := Agent{p: *vector.New(r(), r()), v: *vector.New(r(), r()), r: math.Abs(r())}
			v := c.constructor(a, b)

			t.ResetTimer()
			for i := 0; i < t.N; i++ {
				v.check()
			}
		})
	}
}
