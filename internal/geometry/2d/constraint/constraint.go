package constraint

import (
	"github.com/downflux/go-geometry/2d/constraint"
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

func (c C) C() constraint.C { return c.c }
func (c C) Mutable() bool   { return c.mutable }
