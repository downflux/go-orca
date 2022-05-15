package opt

import (
	"github.com/downflux/go-orca/agent"
)

type O struct {
	// Agent is the input, moving agent for which we need to generate an
	// ORCA plane. This is the agent "A" for ORCA(A, B) in van den Berg et
	// al. (2011).
	Agent agent.A

	// Tau is the lookahead horizon for the ORCA plane, and is in units of
	// time.
	Tau float64

	// TODO(minkezhang): Add support for Weighting and VOpt inputs.
}
