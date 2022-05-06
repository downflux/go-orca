package segment

import (
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/region"
)

var _ region.R = S{}

type O struct {
	P    vector.V
	D    vector.V
	TMin float64
	TMax float64
}

func New(o O) *S {
	return &S{
		s: *segment.New(
			*line.New(o.P, o.D),
			o.TMin,
			o.TMax,
		),
	}
}

type S struct {
	s segment.S
}

func (r S) R() []segment.S { return []segment.S{r.s} }
