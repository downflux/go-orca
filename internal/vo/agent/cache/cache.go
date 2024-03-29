// package cache specifies the truncaged velocity obstacle of A induced by B;
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
package cache

import (
	"math"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/geometry/2d/cone"
	"github.com/downflux/go-orca/internal/vo/agent/cache/domain"
	"github.com/downflux/go-orca/internal/vo/agent/opt"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

const (
	minTau = 1e-3
)

type O struct {
	Obstacle agent.A
	Agent    agent.A
	Tau      float64
	Weight   opt.Weight
	VOpt     opt.VOpt
}

type VO struct {
	agent    agent.A
	obstacle agent.A

	// weight is the relative slack u in the ORCA plane that will be taken
	// up by the input agent (i.e., not the obstacle).
	weight opt.Weight
	vopt   opt.VOpt

	// base is the scaled, characteristic circle of the truncated velocity
	// object.
	base hypersphere.C

	// The cone cache is set at construct time, and is only set for
	// non-collision domains.
	cone cone.C

	// tau is a scalar determining the bottom vertex of the truncated VO;
	// large 𝜏 forces the bottom of the VO closer to the origin. When 𝜏 is
	// infinite, the VO generated is a cone with a point vertex (and the
	// agents cannot move towards one another at all).
	//
	// We interpret 𝜏 as a time scalar -- the larger 𝜏 is, the further ahead
	// in time we can guarantee no collisions (assuming all agents adjust
	// their velocities using ORCA).
	//
	// Note 𝜏 should be roughly on the scale of the input velocities and
	// agent sizes, i.e. if agents are moving at a scale of 100 m/s and are
	// around a meter wide, we should set 𝜏 to ~1 (vs. 1e10).
	//
	// Also note 𝜏 is dependent on how often we will be calling ORCA -- if
	// VO objects are generated once every second, but we only guarantee no
	// collisions for half a second, then collisions may occur.
	tau float64

	// We cache some fields to make things zoom-zoom.
	pIsCached      bool
	wIsCached      bool
	rIsCached      bool
	lIsCached      bool
	vIsCached      bool
	betaIsCached   bool
	thetaIsCached  bool
	domainIsCached bool
	pCache         vector.V
	wCache         vector.V
	lCache         line.L
	vCache         vector.V
	rCache         float64
	betaCache      float64
	thetaCache     float64
	domainCache    domain.D
}

func New(o O) (*VO, error) {
	if o.Tau < minTau {
		return nil, status.Errorf(codes.OutOfRange, "invalid minimum lookahead timestep")
	}

	// c defines the truncation circle.
	c := *hypersphere.New(vector.Scale(1/o.Tau, p(o.Agent, o.Obstacle)), r(o.Agent, o.Obstacle)/o.Tau)

	// We cannot construct a valid cone if the two agents are overlapping,
	// i.e. in the collision domain.
	d, err := cone.New(c)
	if err != nil {
		d = &cone.C{}
	}

	return &VO{
		agent:    o.Agent,
		obstacle: o.Obstacle,
		base:     c,
		cone:     *d,
		tau:      o.Tau,
		weight:   o.Weight,
		vopt:     o.VOpt,
	}, nil
}

// ORCA returns the half-plane of permissable velocities for an agent, given the
// an agent constraint.
func (vo *VO) ORCA() (hyperplane.HP, error) {
	result, err := vo.preprocess()
	if err != nil {
		return hyperplane.HP{}, err
	}
	return *hyperplane.New(
		vector.Add(vo.vopt(vo.agent), vector.Scale(float64(vo.weight), result.U)),
		result.N,
	), nil
}

type result struct {
	U vector.V
	N vector.V
}

func (vo *VO) preprocess() (result, error) {
	switch d := vo.domain(); d {
	case domain.Collision:
		fallthrough
	case domain.Circle:
		tr := vo.r()
		tw := vo.w()

		if d == domain.Collision {
			tr = r(vo.agent, vo.obstacle) / minTau
			tw = w(vo.agent, vo.obstacle, minTau)
		}

		u := vector.Scale(tr/vector.Magnitude(tw)-1, tw)
		n := vector.Unit(vector.Scale(
			map[bool]float64{
				false: 1,
				true:  -1,
			}[vector.SquaredMagnitude(tw) > tr*tr],
			u,
		))

		return result{
			U: u,
			N: n,
		}, nil
	case domain.Right:
		fallthrough
	case domain.Left:
		l := vo.l()

		// The distance u between the relative velocity v and the
		// tangent line ℓ is defined as the vector difference of the
		// vector projection of v onto ℓ to v.
		//
		// The vector projection of v onto ℓ is defined as
		//
		//   v' = cℓ / ||ℓ||
		//
		// where
		//
		//   c = ||v|| * cos(𝜃)
		//     = v • ℓ / ||ℓ||
		//
		// 𝜃 is the usual angle between the two vectors.
		v := vector.Scale(
			vector.Dot(vo.v(), l.D())/vector.SquaredMagnitude(l.D()),
			l.D(),
		)
		u := vector.Sub(v, vo.v())
		n := vector.Unit(vector.Scale(
			// We check the side of v compared to the projected edge
			// ℓ, with the convention that if v is to the "left" of
			// ℓ, we chose n to be anti-parallel to u.
			//
			// N.B.: The "right" leg is represented anti-parallel to
			// the orientation, and therefore already has an
			// implicit negative sign attached, allowing the
			// following determinant to be a continuous calculation
			// from one leg to the other.
			map[bool]float64{
				false: 1,
				true:  -1,
			}[vector.Determinant(l.D(), vo.v()) > 0],
			u,
		))
		return result{
			U: u,
			N: n,
		}, nil
	default:
		return result{}, status.Errorf(codes.Internal, "invalid VO projection %v", d)
	}
}

// domain returns the indicated edge of the truncated VO that is closest to w.
func (vo *VO) domain() domain.D {
	if !vo.domainIsCached {
		vo.domainIsCached = true

		vo.domainCache = func() domain.D {
			beta, err := vo.beta()
			// Retain parity with RVO2 behavior.
			if err != nil {
				return domain.Collision
			}

			theta, err := vo.theta()
			// Retain parity with RVO2 behavior.
			if err != nil {
				return domain.Right
			}

			if theta < beta || math.Abs(2*math.Pi-theta) < beta {
				return domain.Circle
			}

			if theta < math.Pi {
				return domain.Left
			}

			return domain.Right
		}()
	}
	return vo.domainCache
}

// n returns the outward normal vector of u -- that is, if u is pointed towards
// the internal of VO, n should be anti-parallel to u.
func (vo *VO) n() (vector.V, error) {
	result, err := vo.preprocess()
	if err != nil {
		return vector.V{}, err
	}
	return result.N, nil
}

// u returns the calculated vector difference between the relative velocity v
// and the closest part of the VO, pointing into the VO edge.
func (vo *VO) u() (vector.V, error) {
	result, err := vo.preprocess()
	if err != nil {
		return vector.V{}, err
	}
	return result.U, nil
}

// r calculates the radius of the unscaled velocity object.
func (vo *VO) r() float64 {
	if !vo.rIsCached {
		vo.rIsCached = true
		vo.rCache = r(vo.agent, vo.obstacle)
	}
	return vo.rCache
}

// l calulates the domain-aware leg of the tangent line. That is, if u projects
// onto the right leg, ℓ is the tangent line t rotated by -2𝛽, i.e. flipped
// about p.
//
// N.B.: This function must only be called when not in the collision domain.
func (vo *VO) l() line.L {
	if !vo.lIsCached {
		vo.lIsCached = true

		l := vo.cone.L()
		if vo.domain() == domain.Right {
			l = vo.cone.R()
		}
		// ℓ is calculated based on the truncated circle; we are
		// artificially scaling the tangent leg by the scaling factor to
		// match the official RVO2 implementation.
		vo.lCache = *line.New(
			vector.Scale(vo.tau, l.P()),
			vector.Scale(vo.tau, l.D()),
		)
	}
	return vo.lCache
}

// p calculates the center of the unscaled circle of the velocity object -- this
// vector does not take the time scalar 𝜏 into account.
func (vo *VO) p() vector.V {
	if !vo.pIsCached {
		vo.pIsCached = true
		vo.pCache = p(vo.agent, vo.obstacle)
	}
	return vo.pCache
}

// v calculates the relative velocity between a and b.
func (vo *VO) v() vector.V {
	if !vo.vIsCached {
		vo.vIsCached = true
		vo.vCache = v(vo.agent, vo.obstacle)
	}
	return vo.vCache
}

// w calculates the relative velocity between the agent and obstacle, centered
// on the truncation circle.
func (vo *VO) w() vector.V {
	if !vo.wIsCached {
		vo.wIsCached = true
		vo.wCache = w(vo.agent, vo.obstacle, vo.tau)
	}
	return vo.wCache
}

// beta returns the complementary angle between ℓ and p, i.e. the angle
// boundaries at which u should be directed towards the circular bottom of the
// truncated VO.
//
// Note that while 𝛽 is independent of the scaling factor 𝜏.
//
// Returns:
//
//	Angle in radians between 0 and π; w is bound by 𝛽 if -𝛽 < 𝜃 < 𝛽.
func (vo *VO) beta() (float64, error) {
	if !vo.betaIsCached {
		// Check for collisions between agents -- i.e. the combined radii
		// should be greater than the distance between the agents.
		if p := vector.Magnitude(vo.p()); vo.r() > p || epsilon.Within(p, vo.r()) {
			return 0, status.Errorf(codes.OutOfRange, "cannot find the tangent VO angle of colliding agents")
		}

		vo.betaIsCached = true
		vo.betaCache = vo.cone.Beta()
	}
	return vo.betaCache, nil
}

// theta returns the angle between w and p; this can be compared to 𝛽 to
// determine which "edge" of the truncated VO is closest to w.
//
// Note that
//
//  1. w • p   = ||w|| ||p|| cos(𝜃), and
//  2. ||w x p|| = ||w|| ||p|| sin(𝜃)
//
// vo.p() is defined as vo.obstacle.P() - vo.agent.P(); however, we want 𝜃 = 0 when w is
// pointing towards the origin -- that is, opposite the direction of p.
// Therefore, we flip p in our calculations here.
//
// Returns:
//
//	Angle between w and -p in the range [0, 2π).
func (vo *VO) theta() (float64, error) {
	if vector.SquaredMagnitude(vo.w()) == 0 || vector.SquaredMagnitude(vo.p()) == 0 {
		return 0, status.Errorf(codes.OutOfRange, "cannot find the incident angle between w and p for 0-length vectors")
	}

	if !vo.thetaIsCached {
		vo.thetaIsCached = true

		p := vector.Scale(-1, vo.p())

		// w • p
		dotWP := vector.Dot(vo.w(), p)

		// ||w x p||
		crossWP := vector.Determinant(vo.w(), p)

		// ||w|| ||p||
		wp := vector.Magnitude(vo.w()) * vector.Magnitude(p)

		// cos(𝜃) = cos(-𝜃) -- we don't know if 𝜃 lies to the "left" or
		// "right" of 0.
		//
		// Occasionally due to rounding errors, domain here is slightly
		// larger than 1; other bounding issues are prevented with the
		// check on |w| and |p| above, and we are safe to cap the domain
		// here.
		theta := math.Acos(math.Min(1, dotWP/wp))

		// Use sin(𝜃) = -sin(-𝜃) to check the orientation of 𝜃.
		orientation := crossWP/wp > 0
		if !orientation {
			// 𝜃 < 0; shift by 2π radians.
			theta = 2*math.Pi - theta
		}

		vo.thetaCache = theta
	}

	return vo.thetaCache, nil
}

// v is a utility function calculating the relative velocities between two
// agents.
//
// Note that the relative velocity here is oriented from b.V to a.V.
func v(a agent.A, b agent.A) vector.V { return vector.Sub(a.V(), b.V()) }

// r is a utility function calculating the radius of the untruncated VO circle.
func r(a agent.A, b agent.A) float64 { return a.R() + b.R() }

// p is a utility function calculating the relative position vector between two
// agents. p is in position space, and as such, is not directly scaled by the
// truncation factor.
//
// Note the relative position is directed from a.P to b.P.
func p(a agent.A, b agent.A) vector.V { return vector.Sub(b.P(), a.P()) }

// w is a utility function calculating the relative velocity between a and b,
// centered on the truncation circle.
func w(a agent.A, b agent.A, tau float64) vector.V {
	return vector.Sub(v(a, b), vector.Scale(1/tau, p(a, b)))
}
