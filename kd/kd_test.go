package kd

import (
	"testing"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/google/go-cmp/cmp"

	v2d "github.com/downflux/go-geometry/2d/vector"
	agentimpl "github.com/downflux/go-orca/internal/agent"
)

var (
	_ agent.A = &agentimpl.A{}
	_ point.P = &p{}
	_ P       = &p{}
)

type p agentimpl.A

func (p *p) P() vector.V { return vector.V(agentimpl.A(*p).P()) }
func (p *p) A() agent.A {
	a := agentimpl.A(*p)
	return &a
}

func TestKNN(t *testing.T) {
	type config struct {
		name string
		ps   []P
		p    vector.V
		n    int
		want []P
	}
	testConfigs := []config{
		func() config {
			q := p(*agentimpl.New(agentimpl.O{
				P: *v2d.New(1, 2),
			}))
			return config{
				name: "Simple",
				ps:   []P{&q},
				p:    *vector.New(1, 2),
				n:    1,
				want: []P{&q},
			}
		}(),
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			ps := make([]point.P, 0, len(c.ps))
			for _, p := range c.ps {
				ps = append(ps, point.P(p))
			}

			kdtree, err := kd.New(ps)
			if err != nil {
				t.Errorf("New() = _, %v, want = _, %v", err, nil)
			}

			got, err := KNN(Lift(kdtree), c.p, c.n)
			if err != nil {
				t.Errorf("KNN() = _, %v, want = _, %v", err, nil)
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					p{},
					hypersphere.C{},
				),
			); diff != "" {
				t.Errorf("KNN mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
