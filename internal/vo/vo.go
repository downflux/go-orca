// Pacakge vo defines a velocity obstacle object interface for go-orca.
//
// A VO object takes as input a moving agent and a lookahead time, and returns a
// velocity constraint half-plane.
package vo

import (
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-orca/internal/vo/opt"
)

type VO interface {
	// ORCA is a hyperplane in 2D ambient space of the form HP(p, n), i.e.
	// the boundary of the plane passes through p, and faces "into" the
	// planar normal n. We consider points (i.e. velocities) in the plane to
	// be feasible values for the VO struct.
	//
	// More concisely, if HP.In(v), then v is a permissible agent velocity
	// for the current simulation snapshot.
	ORCA(o opt.O) hyperplane.HP
}
