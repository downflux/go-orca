package vo

import (
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

type Agent interface {
	// P returns the current location of the agent.
	P() vector.V

	// V returns the current velocity vector of the agent.
	V() vector.V

	// R returns the characteristic size of the agent.
	R() float64

	// G returns the preferred velocity vector of the agent, e.g. a vector
	// which points to the next waypoint node, with the maximum
	// speed of the agent.
	G() vector.V

	// S returns the maximum speed of the agent.
	S() float64
}

type VO interface {
	ORCA() (plane.HP, error)
}
