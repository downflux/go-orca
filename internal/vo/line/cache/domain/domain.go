package domain

type D int

const (
	// Collision p-domains indicate a physical overlap between the obstacle
	// and the agent.

	CollisionLeft D = iota
	CollisionRight
	CollisionLine

	Normal
)
