package point

import (
	"github.com/downflux/orca/geometry/vector"
)

type P struct {
	v    vector.V
	hash string
}

func (p P) Hash() string { return p.hash }
func (p P) V() vector.V  { return p.v }

func New(v vector.V, hash string) *P { return &P{v: v, hash: hash} }
