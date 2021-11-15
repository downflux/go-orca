package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type A interface {
	// P returns the current location of the agent.
	P() vector.V

	// V returns the current velocity vector of the agent.
	V() vector.V

	// R returns the characteristic size of the agent.
	R() float64

	// T returns the target (read: preferred) velocity vector of the agent,
	// e.g. a vector which points to the next waypoint node, with the
	// maximum speed of the agent.
	T() vector.V

	// S returns the maximum speed of the agent.
	S() float64
}
