package domain

type D int

const (
	// Collision p-domains indicate a physical overlap between the obstacle
	// and the agent.

	CollisionLeft D = iota
	CollisionRight
	CollisionLine

	// Linear v-domains indicate the velocity vector w lies closest to the
	// straight lines of the line VO.

	Left
	LeftCircle
	Right
	RightCircle
	Line
)

func (d D) String() string {
	v, ok := map[D]string{
		CollisionLeft:  "COLLISION_LEFT",
		CollisionRight: "COLLISION_RIGHT",
		CollisionLine:  "COLLISION_LINE",
		Left:           "LEFT",
		LeftCircle:     "LEFT_CIRCLE",
		Right:          "RIGHT",
		RightCircle:    "RIGHT_CIRCLE",
		Line:           "LINE",
	}[d]
	if !ok {
		return "UNKNOWN"
	}
	return v
}
