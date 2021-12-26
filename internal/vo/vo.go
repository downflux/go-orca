// Pacakge vo defines a velocity obstacle object interface for go-orca.
//
// A VO object takes as input a moving agent and a lookahead time, and returns a
// velocity constraint half-plane.
package vo

import (
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-orca/agent"
)

type VO interface {
	ORCA(a agent.A, tau float64) hyperplane.HP
}
