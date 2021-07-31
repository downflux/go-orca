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

func (vo Reference) ORCA() (vector.V, error) {
	switch d := vo.check(); d {
	case Collision:
		fallthrough
	case Circle:
		tr := vo.r()
		tw := vo.w()

		if d == Collision {
			tr = r(vo.a, vo.b, minTau)
			tw = w(vo.a, vo.b, minTau)
		}

		return vector.Scale(tr-vector.Magnitude(tw), vector.Unit(tw)), nil
	case Right:
		fallthrough
	case Left:
		l := vo.l()
		if d == Right {
			l = *vector.New(-l.X(), l.Y())
		}

		return vector.Sub(vector.Scale(vector.Dot(vo.v(), l), l), vo.v()), nil
	}
	return vector.V{}, status.Error(codes.Unimplemented, "unimplemented function")
}
func (vo Reference) r() float64  { return r(vo.a, vo.b, vo.tau) }
func (vo Reference) p() vector.V { return p(vo.a, vo.b, vo.tau) }
func (vo Reference) w() vector.V { return w(vo.a, vo.b, vo.tau) }
func (vo Reference) v() vector.V { return v(vo.a, vo.b) }

// l calculates the unnormalized left vector of the tangent line from the base
// of p to the edge of the truncation circle. This corresponds to line.direction
// in the RVO2 implementation.
func (vo Reference) l() vector.V {
	tp := p(vo.a, vo.b, 1)
	tr := r(vo.a, vo.b, 1)
	l := math.Sqrt(vector.SquaredMagnitude(tp) - math.Pow(tr, 2))
	return vector.Scale(1/vector.SquaredMagnitude(tp), *vector.New(
		tp.X()*l-tp.Y()*tr,
		tp.X()*tr+tp.Y()*l,
	))
}

func (vo Reference) check() Direction {
	if vector.SquaredMagnitude(vo.p()) <= math.Pow(vo.r(), 2) {
		return Collision
	}

	wp := vector.Dot(vo.w(), vo.p())
	if wp < 0 && math.Pow(wp, 2) > vector.SquaredMagnitude(vo.w())*math.Pow(vo.r(), 2) {
		return Circle
	}

	if vector.Determinant(vo.p(), vo.w()) > 0 {
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
	const tolerance = 1e-10
	a := Agent{p: *vector.New(0, 0), v: *vector.New(0, 0), r: 1}
	b := Agent{p: *vector.New(0, 5), v: *vector.New(1, -1), r: 2}

	testConfigs := []struct {
		name string
		tau  float64
		want vector.V
	}{
		{
			name: "NormalScale",
			tau:  1,
			// This value was determined experimentally.
			want: *vector.New(0.2723931248910011, 1.0895724995640044),
		},
		{
			name: "NormalScaleLargeTau",
			tau:  3,
			// This value was determined experimentally.
			want: *vector.New(0.16000000000000003, 0.11999999999999988),
		},
	}
	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := Reference{a: a, b: b, tau: c.tau}
			if got, err := r.ORCA(); err != nil || !withinV(got, c.want, tolerance) {
				t.Errorf("check() = %v, %v, want = %v, %v", got, err, c.want, nil)
			}
		})
	}
}

// rd returns a random int between [-100, 100).
func rd() float64 { return rand.Float64()*200 - 100 }

// within checks that two numeric values are within a small range of one
// another.
func within(got float64, want float64, tolerance float64) bool {
	return math.Abs(got-want) < tolerance
}

func withinV(got vector.V, want vector.V, tolerance float64) bool {
	return within(got.X(), want.X(), tolerance) && within(got.Y(), want.Y(), tolerance)
}

// TestVODirectionConformance tests that agent-agent VOs will return u in the
// correct direction using the reference implementation as a sanity check.
func TestVODirectionConformance(t *testing.T) {
	const nTests = 1000
	const tolerance = 1e-10
	const delta = 1e-10

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
			a:    Agent{p: *vector.New(rd(), rd()), v: *vector.New(rd(), rd()), r: math.Abs(rd())},
			b:    Agent{p: *vector.New(rd(), rd()), v: *vector.New(rd(), rd()), r: math.Abs(rd())},
			// A simulation timestep scalar of 0 indicates the
			// simulation will never advance to the next snapshot,
			// which is a meaningless case (and will produce
			// boundary condition errors in our implementation).
			tau: math.Abs(rd()) + delta,
		})
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			v, err := New(c.a, c.b, c.tau)
			if err != nil {
				t.Fatalf("New() returned a non-nil error: %v", err)
			}
			r := Reference{a: c.a, b: c.b, tau: float64(c.tau)}

			t.Run("Direction", func(t *testing.T) {
				want := r.check()
				if got := v.check(); got != want {
					beta, _ := v.beta()
					theta, _ := v.theta()

					// Disregard rounding errors around where 𝜃 ~ 𝛽.
					if got == Circle && (!within(beta, theta, tolerance) || !within(2*math.Pi-theta, beta, tolerance)) {
						return
					}
					t.Errorf("check() = %v, want = %v", got, want)
				}
			})
			t.Run("ORCA", func(t *testing.T) {
				want, err := r.ORCA()
				if err != nil {
					t.Fatalf("ORCA() returned error: %v", err)
				}
				got, err := v.ORCA()
				if err != nil {
					t.Fatalf("ORCA() returned error: %v", err)
				}

				if !withinV(got, want, tolerance) {
					t.Errorf("ORCA(%v, %v) = %v, want = %v", v.check(), r.check(), got, want)
				}
			})
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
			a := Agent{p: *vector.New(rd(), rd()), v: *vector.New(rd(), rd()), r: math.Abs(rd())}
			b := Agent{p: *vector.New(rd(), rd()), v: *vector.New(rd(), rd()), r: math.Abs(rd())}
			v := c.constructor(a, b)

			t.ResetTimer()
			for i := 0; i < t.N; i++ {
				v.check()
			}
		})
	}
}
