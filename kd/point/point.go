package point

import (
	"github.com/downflux/orca/geometry/vector"
)

// TODO(minkezhang): Refactor using generics.
type P interface {
	V() vector.V

	Equal(q P) bool
}
