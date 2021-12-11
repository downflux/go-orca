package agent

import (
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

func (a *A) P() vector.V { return a.p }
func (a *A) T() vector.V {
	v := vector.Sub(a.G(), a.P())
	if epsilon.Within(vector.SquaredMagnitude(v), 0) {
		return *vector.New(0, 0)
	}
	return vector.Scale(a.S(), vector.Unit(v))
}
func (a *A) V() vector.V     { return a.v }
func (a *A) G() vector.V     { return a.g }
func (a *A) R() float64      { return a.r }
func (a *A) S() float64      { return a.s }
func (a *A) SetV(v vector.V) { a.v = v }
func (a *A) SetP(v vector.V) { a.p = v }
