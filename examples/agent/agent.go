package agent

import (
	"math"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/agent"
)

var _ agent.A = &A{}

type O struct {
	P vector.V
	G vector.V
	S float64
	R float64
}

func New(o O) *A {
	a := &A{
		p: o.P,
		g: o.G,
		s: o.S,
		r: o.R,
	}
	a.SetV(a.T())
	return a
}

type A struct {
	p vector.V
	v vector.V
	g vector.V

	s float64
	r float64
}

// S returns the maximum speed of the agent. We are setting the agent speed to
// be invariant -- this means agents can still move to avoid other agents even
// after arriving at the destination.
func (a *A) S() float64 { return a.s }

// T returns the target velocity vector of the agent. In the demos provided, we
// are setting the target to stop when "close enough" to the actual goal to
// remove end-state jitter while preserving the flexibility agent to still
// navigate to avoid collisions.
func (a *A) T() vector.V {
	d := vector.Sub(a.G(), a.P())
	m := vector.SquaredMagnitude(d)
	if epsilon.Absolute(1e-5).Within(m, 0) {
		d = *vector.New(0, 0)
	}
	return vector.Scale(math.Min(a.S(), a.S()*m), d)
}

func (a *A) P() vector.V     { return a.p }
func (a *A) V() vector.V     { return a.v }
func (a *A) G() vector.V     { return a.g }
func (a *A) R() float64      { return a.r }
func (a *A) SetV(v vector.V) { a.v = v }
func (a *A) SetP(v vector.V) { a.p = v }
