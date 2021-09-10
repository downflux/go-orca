package circle

import (
	"github.com/downflux/orca/geometry/vector"
)

type C struct {
	r float64
	p vector.V
}

func New(p vector.V, r float64) *C {
	return &C{r: r, p: p}
}

func (c C) R() float64  { return c.r }
func (c C) P() vector.V { return c.p }
