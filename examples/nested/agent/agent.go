package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"

	demoagent "github.com/downflux/go-orca/examples/agent"
)

var _ agent.A = &A{}

type A struct {
	parent agent.A

	*demoagent.A
}

type O struct {
	demoagent.O

	Parent agent.A
}

func New(o O) *A {
	return &A{
		A: demoagent.New(demoagent.O{
			P: o.P,
			G: o.G,
			S: o.S,
			R: o.R,
		}),
		parent: o.Parent,
	}
}

func (a *A) T() vector.V {
	if a.parent == nil {
		return a.A.T()
	}
	return vector.Add(
		a.A.T(),
		vector.Scale(0.1, vector.Sub(a.A.T(), a.parent.T())),
	)
}

func (a *A) S() float64 {
	if a.parent == nil {
		return a.A.S()
	}
	if epsilon.Within(a.parent.G(), a.parent.
	return a.parent.S()
}
