package domain

type D string

const (
	// Collision p-domains indicate a physical overlap between the obstacle
	// and the agent.

	CollisionLeft  D = "COLLISION_LEFT"
	CollisionRight   = "COLLISION_RIGT"
	CollisionLine    = "COLLISION_LINE"

	// Circle v-domains indicate the velocity vector w lies closest to the
	// turncation circles.

	CircleLeft  = "CIRCLE_LEFT"
	CircleRight = "CIRCLE_RIGHT"

	// Linear v-domains indicate the velocity vector w lies closest to the
	// straight lines of the line VO.

	Left  = "LEFT"
	Right = "RIGHT"
	Line  = "LINE"
)
