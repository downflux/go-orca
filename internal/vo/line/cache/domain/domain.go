package domain

type P int

const (
	CollisionLeft P = iota
	CollisionRight
	CollisionLine

	ObliqueLeft
	ObliqueRight

	Normal
)
