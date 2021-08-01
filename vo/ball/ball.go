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

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/vo"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

const (
	minTau = 1e-3
)

type Direction string

const (
	Left      Direction = "LEFT"
	Right               = "RIGHT"
	Circle              = "CIRCLE"
	Collision           = "COLLISION"
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
	// around a meter wide, we should set 𝜏 to ~1 (vs. 1e10).
	tau float64

	// We cache some fields to make things zoom-zoom.
	pIsCached    bool
	wIsCached    bool
	rIsCached    bool
	lIsCached    bool
	vIsCached    bool
	betaIsCached bool
	pCache       vector.V
	wCache       vector.V
	lCache       vector.V
	vCache       vector.V
	rCache       float64
	betaCache    float64
}

func New(a, b vo.Agent, tau float64) (*VO, error) {
	if tau <= 0 {
		return nil, status.Errorf(codes.OutOfRange, "invalid minimum lookahead time step")
	}
	return &VO{a: a, b: b, tau: tau}, nil
}

func (vo *VO) ORCA() (plane.HP, error) {
	return plane.HP{}, status.Errorf(codes.Unimplemented, "")
}

func (vo *VO) u() (vector.V, error) {
	switch d := vo.check(); d {
	case Collision:
		fallthrough
	case Circle:
		tr := vo.r()
		tw := vo.w()
		if d == Collision {
			tr = r(vo.a, vo.b, minTau)
			tw = w(vo.a, vo.b, minTau)
		}

		return vector.Scale(tr/vector.Magnitude(tw)-1, tw), nil
	case Right:
		fallthrough
	case Left:
		l := vo.l()
		if d == Right {
			l = *vector.New(-l.X(), l.Y())
		}

		// The distance u between the relative velocity v and the
		// tangent line ℓ is defined as the vector difference of the
		// vector projection of v onto ℓ to v.
		//
		// The vector projection of v onto ℓ is defined as
		//
		// v' = cℓ / ||ℓ||, where
		//
		// c = ||v|| * cos(𝜃)
		//   = v • ℓ / ||ℓ||
		//
		// 𝜃 is the usual angle between the two vectors.
		v := vector.Scale(
			vector.Dot(vo.v(), l)/vector.SquaredMagnitude(l),
			l,
		)
		return vector.Sub(v, vo.v()), nil
	default:
		return vector.V{}, status.Errorf(codes.Internal, "invalid VO projection %v", d)
	}
}

// r calculates the radius of the truncation circle.
func (vo *VO) r() float64 {
	if !vo.rIsCached {
		vo.rIsCached = true
		vo.rCache = r(vo.a, vo.b, vo.tau)
	}
	return vo.rCache
}

// l calculates the left vector of the tangent line segment from the base of p
// to the edge of the truncation circle.
//
// N.B.: The direction of ℓ can be calculated by rotating p about the origin by
// 𝛼 := π / 2 - 𝛽, and scaling up via ||p|| ** 2 = ||ℓ|| ** 2 + r ** 2.
//
// Note that ℓ, p, and a third leg with length r form a right triangle. Because
// of this, We know cos(𝛼) = ||ℓ|| / ||p|| and sin(𝛼) = r / ||p||. These can be
// substituted directly to the rotation matrix:
//
// ℓ ~ V{ x: p.x * cos(𝛼) - p.y * sin(𝛼),
//        y: p.x * sin(𝛼) + p.y * cos(𝛼) }
//
// See design doc for more information.
func (vo *VO) l() vector.V {
	if !vo.lIsCached {
		vo.lIsCached = true
		l := math.Sqrt(vector.SquaredMagnitude(vo.p()) - math.Pow(vo.r(), 2))
		vo.lCache = vector.Scale(
			l,
			vector.Unit(
				*vector.New(
					vo.p().X()*l-vo.p().Y()*vo.r(),
					vo.p().X()*vo.r()+vo.p().Y()*l,
				),
			),
		)
	}
	return vo.lCache
}

// p calculates the center of the truncation circle. Geometrically, this is the
// relative position of b from a, scaled by 𝜏.
func (vo *VO) p() vector.V {
	if !vo.pIsCached {
		vo.pIsCached = true
		vo.pCache = p(vo.a, vo.b, vo.tau)
	}
	return vo.pCache
}

// v calculates the relative velocity between a and b.
func (vo *VO) v() vector.V {
	if !vo.vIsCached {
		vo.vIsCached = true
		vo.vCache = vector.Sub(vo.a.V(), vo.b.V())
	}
	return vo.vCache
}

// w calculates the relative velocity between a and b, centered on the truncation circle.
func (vo *VO) w() vector.V {
	if !vo.wIsCached {
		vo.wIsCached = true
		vo.wCache = vector.Sub(vo.v(), vo.p())
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
//   Angle in radians between 0 and 2π, between w and -p.
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

// v is a utility function calculating the relative velocities between two
// agents.
//
// Note that the relative velocity here is oriented from b.V to a.V.
func v(a vo.Agent, b vo.Agent) vector.V { return vector.Sub(a.V(), b.V()) }

// r is a utility function calculating the radius of the truncated VO circle.
func r(a vo.Agent, b vo.Agent, tau float64) float64 { return (a.R() + b.R()) / tau }

// p is a utility function calculating the relative position vector between two
// agents, scaled to the center of the truncated circle.
//
// Note the relative position is oriented from a.P to b.P.
func p(a vo.Agent, b vo.Agent, tau float64) vector.V {
	return vector.Scale(1/tau, vector.Sub(b.P(), a.P()))
}

// w is a utility function calculating the relative velocity between a and b,
// centered on the truncation circle.
func w(a vo.Agent, b vo.Agent, tau float64) vector.V {
	return vector.Sub(v(a, b), p(a, b, tau))
}
