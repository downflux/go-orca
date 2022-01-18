package vo

import (
	"github.com/downflux/go-geometry/2d/hyperplane"
)

type VO interface {
	ORCA() (hyperplane.HP, error)
}
