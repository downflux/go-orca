// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

type HP struct {
	n vector.V
	p vector.V
}

func New(p vector.V, n vector.V) *HP {
	return &HP{n: n, p: p}
}
