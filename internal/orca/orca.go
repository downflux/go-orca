// Package orca describes some minimal utilities used by the external orca
// package, but is shared across multiple packages in e.g. the demo.
package orca

import (
	"math"

	"github.com/downflux/go-orca/agent"
)

// R defines neighboring radius around which objects will have an influence on
// the given agent.
func R(a agent.A, tau float64) float64 {
	return math.Min(
		math.Max(
			100*tau*a.S(),
			// We want to ensure at least one agent in a collision
			// system will see its neighbors -- this ensures the
			// largest agent will always move to avoid smaller
			// agents.
			3*a.R(),
		),
		6*a.R(),
	)
}
