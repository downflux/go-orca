package solver

import (
	"github.com/downflux/orca/geometry/lp/constraint"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/optimize/convex/lp"
)

type S struct {
	cs        []constraint.C
	tolerance float64
}

func New(cs []constraint.C, tolerance float64) *S {
	return &S{
		cs:        cs,
		tolerance: tolerance,
	}
}

func (s *S) AddConstraint(c constraint.C) {
	s.cs = append(s.cs, c)
}

func (s *S) Minimize(c []float64) ([]float64, error) {
	if len(c) == 0 {
		return nil, nil
	}
	a := mat.NewDense(len(s.cs), s.cs[0].Dimension(), nil)
	var b []float64

	for i, c := range s.cs {
		a.SetRow(i, c.A())
		b = append(b, c.B())
	}

	_, x, err := lp.Simplex(c, a, b, s.tolerance, nil)
	if err != nil {
		return nil, err
	}
	return x, err
}

func (s *S) Maximize(c []float64) ([]float64, error) {
	for i, cx := range c {
		c[i] = -cx
	}
	return s.Minimize(c)
}
