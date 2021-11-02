package vo

import (
	"github.com/downflux/orca/geometry/plane"
)

type VO interface {
	ORCA() (plane.HP, error)
}
