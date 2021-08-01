package vo

import (
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

type Agent interface {
	P() vector.V
	V() vector.V
	R() float64
}

type VO interface {
	ORCA() (plane.HP, error)
}
