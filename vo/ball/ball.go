// package ball specifies the truncaged velocity obstacle of A induced by B;
// that is, an object which tests for what velocies are permissible by A to
// choose that will not hit B.
//
// Geometrically, a truncated VO is a rounded cone, i.e. the "bottom" of the
// cone is bounded by a circle (the "truncation circle").
//
// The radius of this truncation circle is proportional to the combined radii of
// the agents, and inversely proportional to a scaling factor 𝜏; as we increase
// 𝜏, the shape of VO approaches that of an untruncated cone, and the amount of
// "forbidden" velocities for the agents increases.
//
// We interpret the scaling factor 𝜏 as the simulation lookahead time -- as 𝜏
// increases, agents are more responsive to each other at larger distances;
// however, if 𝜏 is too large, agent movements stop resembling "reasonable"
// human behavior and may veer off-course too early.
//
// TODO(minkezhang): Add design doc link.
package ball

import (
	"math"

	"github.com/downflux/orca/vector"
	"github.com/downflux/orca/vo"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

const (
	maxTimeResolution = 100
)

type Direction string

const (
	Left   Direction = "LEFT"
	Right            = "RIGHT"
	Circle           = "CIRCLE"

	// TODO(minkezhang): Handle this case gracefully.
	Collision = "COLLISION"
)

type VO struct {
	a vo.Agent
	b vo.Agent

	// tau is a scalar determining the bottom vertex of the truncated VO;
	// large 𝜏 forces the bottom of the VO closer to the origin. When tau is
	// infinite, the VO generated is a cone with a point vertex.
	//
	// Note 𝜏 should be roughly on the scale of the input velocities and
	// agent sizes, i.e. if agents are moving at a scale of 100 m/s and are
	// of size meters, we should set 𝜏 to ~1 (vs. 1e10).
	tau float64

	// We cache some fields to make things zoom-zoom.
	pIsCached bool
	wIsCached bool
	rIsCached bool
	betaIsCached bool
	pCache    vector.V
	wCache    vector.V
	rCache    float64
	betaCache float64
}

func New(a, b vo.Agent, tau float64) (*VO, error) {
	if tau <= 0 {
		return nil, status.Errorf(codes.OutOfRange, "invalid minimum lookahead time step")
	}
	return &VO{a: a, b: b, tau: tau}, nil
}

// TODO(minkezhang): Implement.
func (vo *VO) ORCA() vector.V {
	switch d := vo.check(); d {
	case Circle:
		return vector.Scale(vo.r() / vector.Magnitude(vo.w()) - 1, vo.w())
	case Collision:
		w := vector.Sub(
			vector.Sub(vo.a.V(), vo.b.V()),
			vector.Scale(1 / maxTimeResolution, vector.Sub(vo.b.P(), vo.a.P())),
		)
		return vector.Scale((vo.a.R() + vo.b.R()) / maxTimeResolution / vector.Magnitude(w) - 1, w)
	case Left:
	case Right:
	}
	return vector.V{}
}

// r calculates the radius of the truncation circle.
func (vo *VO) r() float64 {
	if !vo.rIsCached {
		vo.rIsCached = true
		vo.rCache = (vo.a.R() + vo.b.R()) / vo.tau
	}
	return vo.rCache
}

// l calculates the length of the tangent line segment from the start of p to the
// edge of the truncation circle.
func (vo *VO) l() float64 { return math.Sqrt(vector.SquaredMagnitude(vo.p()) - math.Pow(vo.r(), 2)) }

// p calculates the center of the truncation circle. Geometrically, this is the
// relative position of b from a, scaled by 𝜏.
func (vo *VO) p() vector.V {
	if !vo.pIsCached {
		vo.pIsCached = true
		vo.pCache = vector.Scale(1/vo.tau, vector.Sub(vo.b.P(), vo.a.P()))
	}
	return vo.pCache
}

// w calculates the relative velocity between a and b, centered on the truncation circle.
func (vo *VO) w() vector.V {
	if !vo.wIsCached {
		vo.wIsCached = true
		vo.wCache = vector.Sub(vector.Sub(vo.a.V(), vo.b.V()), vo.p())
	}
	return vo.wCache
}

// beta returns the complementary angle between l and p, i.e. the angle
// boundaries at which u should be directed towards the circular bottom of the
// truncated VO.
//
// Returns:
//   Angle in radians between 0 and π; w is bound by 𝛽 if -𝛽 < 𝜃 < 𝛽.
func (vo *VO) beta() (float64, error) {
	// Check for collisions between agents -- i.e. the combined radii
	// should be greater than the distance between the agents.
	//
	// Note that r and p are both scaled by 𝜏 here, and as such, cancels
	// out, giving us the straightforward conclusion that we should be able
	// to detect collisions independent of the lookahead time.
	if math.Pow(vo.r(), 2) >= vector.SquaredMagnitude(vo.p()) {
		return 0, status.Errorf(codes.OutOfRange, "cannot find the tangent VO angle of colliding agents")
	}

	if !vo.betaIsCached {
		vo.betaIsCached = true
		// Domain error when Acos({x | x > 1}).
		vo.betaCache = math.Acos(vo.r() / vector.Magnitude(vo.p()))
	}

	return vo.betaCache, nil
}

// theta returns the angle between w and p; this can be compared to 𝛽 to
// determine which "edge" of the truncated VO is closest to w.
//
// Note that
//
// 1.   w • p   = ||w|| ||p|| cos(𝜃), and
// 2. ||w x p|| = ||w|| ||p|| sin(𝜃)
//
// vo.p() is defined as vo.b.P() - vo.a.P(); however, we want 𝜃 = 0 when w is
// pointing towards the origin -- that is, opposite the direction of p.
// Therefore, we flip p in our calculations here.
//
// Returns:
//   Angle in radians between 0 and 2π between w and -p.
func (vo *VO) theta() (float64, error) {
	if vector.SquaredMagnitude(vo.w()) == 0 || vector.SquaredMagnitude(vo.p()) == 0 {
		return 0, status.Errorf(codes.OutOfRange, "cannot find the incident angle between w and p for 0-length vectors")
	}

	p := vector.Scale(-1, vo.p())

	// w • p
	dotWP := vector.Dot(vo.w(), p)

	// ||w x p||
	crossWP := vector.Determinant(vo.w(), p)

	// ||w|| ||p||
	wp := vector.Magnitude(vo.w()) * vector.Magnitude(p)

	// cos(𝜃) = cos(-𝜃) -- we don't know if 𝜃 lies to the "left" or "right" of 0.
	//
	// Occasionally due to rounding errors, domain here is slightly larger
	// than 1; other bounding issues are prevented with the check on |w| and
	// |p| above, and we are safe to cap the domain here.
	theta := math.Acos(math.Min(1, dotWP/wp))

	// Use sin(𝜃) = -sin(-𝜃) to check the orientation of 𝜃.
	orientation := crossWP/wp > 0
	if !orientation {
		// 𝜃 < 0; shift by 2π radians.
		theta = 2*math.Pi - theta
	}
	return theta, nil
}

// check returns the indicated edge of the truncated VO that is closest to w.
func (vo *VO) check() Direction {
	beta, err := vo.beta()
	// Retain parity with RVO2 behavior.
	if err != nil {
		return Collision
	}

	theta, err := vo.theta()
	// Retain parity with RVO2 behavior.
	if err != nil {
		return Right
	}

	if theta < beta || math.Abs(2*math.Pi-theta) < beta {
		return Circle
	}

	if theta < math.Pi {
		return Left
	}

	return Right
}
