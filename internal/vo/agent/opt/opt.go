package opt

import (
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
)

const (
	WeightEqual = 0.5
	WeightAll   = 1.0
	WeightNone  = 0.0
)

func VOptV(agent agent.A) vector.V    { return agent.V() }
func VOptZero(agent agent.A) vector.V { return *vector.New(0, 0) }

// Weight is the relative responsibility the input agent needs to take
// for steering away from the obstacle -- for ball-ball interactions,
// this is typically 0.5, but for ball-wall interactions, this is set to
// 1, as walls are considered immovable.
type Weight float64

// VOpt returns the optimal velocity for the input agent, per van den
// Berg et al. (2011). For ball-ball interactions, this is typically set
// to the agent velocity, but for ball-wall interactions, this is set to
// the 0-vector instead.
//
// N.B.: Note that in van den Berg et al., the optimal velocity is also
// used to calculate the vector u which touches the nearest side of the
// VO. Per the RVO2 implementation, this does not appear to actually be
// the case, and VOpt is only used in calculating the base of the ORCA
// plane.
type VOpt func(agent agent.A) vector.V

type O struct {
	Weight Weight
	VOpt   VOpt
}

func Validate(o O) error {
	if o.Weight < 0 || o.Weight > 1 {
		return grpc.Errorf(codes.InvalidArgument, "invalid agent ORCA weighting")
	}

	if o.VOpt == nil {
		return grpc.Errorf(codes.InvalidArgument, "invalid agent optimal velocity function")
	}

	return nil
}
