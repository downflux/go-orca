package vector

import (
	"math"
)

type V struct {
	x, y float64
}

func New(x, y float64) *V { return &V{x: x, y: y} }

func (v V) X() float64 { return v.x }
func (v V) Y() float64 { return v.y }

func Add(v, u V) V               { return V{x: v.x + u.x, y: v.y + u.y} }
func Sub(v, u V) V               { return V{x: v.x - u.x, y: v.y - u.y} }
func Scale(s float64, v V) V     { return V{x: s * v.x, y: s * v.y} }
func Dot(v, u V) float64         { return v.x*u.x + v.y*u.y }
func Determinant(v, u V) float64 { return v.x*u.y - v.y*u.x }

func IsOrthogonal(v, u V) bool     { return Dot(v, u) == 0 }
func SquaredMagnitude(a V) float64 { return Dot(a, a) }
func Magnitude(a V) float64        { return math.Sqrt(SquaredMagnitude(a)) }
func Unit(a V) V                   { return Scale(1/Magnitude(a), a) }
func Rotate(theta float64, v V) V {
	return V{
		x: v.x*math.Cos(theta) - v.y*math.Sin(theta),
		y: v.x*math.Sin(theta) + v.y*math.Cos(theta),
	}
}

func Within(a V, b V, tolerance float64) bool { return Magnitude(Sub(a, b)) < tolerance }
