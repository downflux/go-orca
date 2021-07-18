// package ball specifies the truncaged velocity obstacle of A induced by B;
// that is, an object which tests for what velocies are permissible by A to
// choose that will not hit B.
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

type Direction string

const (
	Left   Direction = "LEFT"
	Right            = "RIGHT"
	Circle           = "CIRCLE"

	// TODO(minkezhang): Handle this case gracefully.
	Collision = "COLLISION"
)

type VO struct {
	a   vo.Agent
	b   vo.Agent
	tau float64 // Minimum timestep is unimplemented.
}

func New(a, b vo.Agent) *VO { return &VO{a: a, b: b} }

// TODO(minkezhang): Implement.
func (vo VO) ORCA() vector.V { return vector.V{} }

// r calculates the combined radius between the two Agent objects.
func (vo VO) r() float64 { return vo.a.R() + vo.b.R() }

// l calculates the length of the tangent line segment from the start of p to the
// edge of the circle of radius r.
func (vo VO) l() float64 { return math.Sqrt(vector.SquaredMagnitude(vo.p()) - math.Pow(vo.r(), 2)) }

// p calculates the relative position of b from agent a.
func (vo VO) p() vector.V { return vector.Sub(vo.b.P(), vo.a.P()) }

// w calculates the relative velocity between a and b, centered on the combined circle.
func (vo VO) w() vector.V { return vector.Sub(vector.Sub(vo.a.V(), vo.b.V()), vo.p()) }

// beta returns the complementary angle between l and p, i.e. the angle
// boundaries at which u should be directed towards the circular bottom of the
// truncated VO.
//
// Returns:
//   Angle in radians between 0 and π; w is bound by 𝛽 if -𝛽 < 𝜃 < 𝛽.
func (vo VO) beta() (float64, error) {
	if math.Pow(vo.r(), 2) >= vector.SquaredMagnitude(vo.p()) {
		return 0, status.Error(codes.OutOfRange, "cannot find the tangent VO angle of colliding balls")
	}

	// Domain error when Acos({x | x > 1}).
	return math.Acos(vo.r() / vector.Magnitude(vo.p())), nil
}

// theta returns the angle between w and -p; this can be compared to theta to
// determine which edge of the truncated VO is closest to w.
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
func (vo VO) theta() (float64, error) {
	if vector.SquaredMagnitude(vo.w()) == 0 || vector.SquaredMagnitude(vo.p()) == 0 {
		return 0, status.Error(codes.OutOfRange, "cannot find the incident angle between w and p for 0-length vectors")
	}

	// w • p
	dotWP := vector.Dot(vo.w(), vector.Scale(-1, vo.p()))

	// ||w x p||
	crossWP := vector.Determinant(vo.w(), vector.Scale(-1, vo.p()))

	// ||w|| ||p||
	wp := vector.Magnitude(vo.w()) * vector.Magnitude(vector.Scale(-1, vo.p()))

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
func (vo VO) check() Direction {
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
