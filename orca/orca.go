package orca

import (
	"github.com/downflux/orca/geometry/lp/constraint"
	"github.com/downflux/orca/geometry/lp/solver"
	"github.com/downflux/orca/vo"
	"github.com/downflux/orca/vo/ball"
)

const tolerance = 1e-10

type ORCA struct {
	agents []vo.Agent
}

func New(agents []vo.Agent, tau float64) *ORCA {
	return &ORCA{
		agents: agents,
	}
}

func (o *ORCA) Step(tau float64) error {
	for _, a := range o.agents {
		s := *solver.New(
			[]constraint.C{
				*constraint.New([]float64{1, 0}, a.G().Y()), // px <= Mx
				*constraint.New([]float64{0, 1}, a.G().X()), // py <= My
			},
			tolerance,
		)

		// TODO(minkezhang): Replace with nearest neighbor filter.
		for _, b := range o.agents {
			if b != a {
				v, err := ball.New(a, b, tau)
				if err != nil {
					return err
				}
				orca, err := v.ORCA()
				if err != nil {
					return err
				}
				s.AddConstraint(orca)
			}
		}
	}
	return nil
}
