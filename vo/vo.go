package vo

import (
	"github.com/downflux/orca/geometry/vector"
)

type Agent interface {
	P() vector.V
	V() vector.V
	R() float64
}

type VO interface {
	ORCA() (vector.V, error)
}
