package region

import (
	"github.com/downflux/go-geometry/2d/segment"
)

// R is a collection of line segments representing physical walls within the
// map. R is immovable, and may either be open or closed.
type R interface {
	R() []segment.S
}
