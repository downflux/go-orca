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
	Right
	Line
)
