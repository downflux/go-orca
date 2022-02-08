package cone

import (
	"math"

	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

type C hypersphere.C

func New(center hypersphere.C) (*C, error) {
	if epsilon.Within(center.R(), 0) || center.R()*center.R() >= vector.SquaredMagnitude(center.P()) {
		return nil, status.Errorf(codes.OutOfRange, "cannot construct truncated cone with given characteristic circle base")
	}
	c := C(center)
	return &c, nil
}

// beta returns the complementary angle between ℓ and p, i.e. the angle
// boundaries at which u should be directed towards the circular bottom of the
// truncated VO.
//
// Returns:
//   Angle in radians between 0 and π; w is bound by 𝛽 if -𝛽 < 𝜃 < 𝛽.
func (c C) beta() float64 {
	hc := hypersphere.C(c)

	return math.Acos(hc.R() / vector.Magnitude(hc.P()))
}

// L calculates the left vector of the tangent line segment from the base of p
// to the edge of the truncation circle.
//
// N.B.: The domain of ℓ can be calculated by rotating p anti-clockwise about
// the origin by
//
//   𝛼 := π / 2 - 𝛽
//
// ℓ may be scaled via
//
//   ||p|| ** 2 = ||ℓ|| ** 2 + r ** 2.
//
// Note that ℓ, p, and a third leg with length r form a right triangle. Because
// of this, We know cos(𝛼) = ||ℓ|| / ||p|| and sin(𝛼) = r / ||p||. These can be
// substituted directly to the rotation matrix:
//
//   ℓ ~ V{ x: p.x * cos(𝛼) - p.y * sin(𝛼),
//          y: p.x * sin(𝛼) + p.y * cos(𝛼) }
//
// See design doc for more information.
func (c C) L() vector.V {
	hc := hypersphere.C(c)

	l := math.Sqrt(
		vector.SquaredMagnitude(hc.P()) - hc.R()*hc.R(),
	)
	return vector.Scale(
		l,
		vector.Unit(
			*vector.New(
				hc.P().X()*l-hc.P().Y()*hc.R(),
				hc.P().X()*hc.R()+hc.P().Y()*l,
			),
		),
	)
}

// R calculates the right vector of the tangent line segment from the base of p
// to the edge of te truncation circle.
func (c C) R() vector.V {
	return vector.Rotate(2*c.beta(), c.L())
}
