package kd

import (
	"github.com/kyroy/kdtree/points/point2d"
	"github.com/downflux/orca/vector/vector"
)

type p struct {
	v vector.V
}

func (p p) Dimensions() int { return 2 }
func (p p) Dimension(i int) {
	if i == 0 { return p.v.X() }
	return p.v.Y()
}

type T struct {
}
