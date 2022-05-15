package constraint

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

type C struct {
	c       constraint.C
	mutable bool
}

func New(c constraint.C, mutable bool) *C {
	return &C{
		c:       c,
		mutable: mutable,
	}
}

func (c C) In(v vector.V) bool { return c.c.In(v) }
func (c C) Mutable() bool      { return c.mutable }
