// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

type HP struct {
	p vector.V
	n vector.V
}

func New(p vector.V, n vector.V) *HP {
	return &HP{n: n, p: p}
}

func (p HP) N() vector.V { return p.n }
func (p HP) P() vector.V { return p.p }
