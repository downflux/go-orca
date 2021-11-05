package vo

import (
	"github.com/downflux/go-geometry/plane"
)

type VO interface {
	ORCA() (plane.HP, error)
}
