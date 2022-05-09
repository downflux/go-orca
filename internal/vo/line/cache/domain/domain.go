package domain

type D string

const (
	// Collision p-domains indicate a physical overlap between the obstacle
	// and the agent.

	CollisionLeft  D = "COLLISION_LEFT"
	CollisionRight   = "COLLISION_RIGHT"
	CollisionLine    = "COLLISION_LINE"

	// Linear v-domains indicate the velocity vector w lies closest to the
	// straight lines of the line VO.

	Left  = "LEFT"
	Right = "RIGHT"
	Line  = "LINE"
)
