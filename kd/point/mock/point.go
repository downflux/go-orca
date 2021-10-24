package point

import (
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/point"
)

type P struct {
	v    vector.V
	hash string
}

func (p P) Equal(q point.P) bool { return p.hash == q.(P).hash }
func (p P) V() vector.V          { return p.v }

func New(v vector.V, hash string) *P { return &P{v: v, hash: hash} }
