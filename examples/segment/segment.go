package segment

import (
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-orca/region"
)

var _ region.R = S{}

type S struct {
	s segment.S
}

func (r S) R() []segment.S { return []segment.S{r.s} }
