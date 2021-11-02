package vector

import (
	"math"
	"testing"
)

func TestAdd(t *testing.T) {
	v := V{1, 1}
	u := V{2, 3}
	want := V{3, 4}
	if got := Add(v, u); got != want {
		t.Errorf("Add() = %v, want = %v", got, want)
	}
}

func TestSub(t *testing.T) {
	v := V{1, 1}
	u := V{2, 3}
	want := V{-1, -2}
	if got := Sub(v, u); got != want {
		t.Errorf("Sub() = %v, want = %v", got, want)
	}
}

func TestScale(t *testing.T) {
	v := V{2, 3}
	s := float64(2)
	want := V{4, 6}
	if got := Scale(s, v); got != want {
		t.Errorf("Scale() = %v, want = %v", got, want)
	}
}

func TestDot(t *testing.T) {
	v := V{1, 1}
	u := V{2, 3}
	want := float64(5)
	if got := Dot(v, u); got != want {
		t.Errorf("Dot() = %v, want = %v", got, want)
	}
}

func TestDeterminant(t *testing.T) {
	v := V{1, 1}
	u := V{2, 3}
	want := float64(1)
	if got := Determinant(v, u); got != want {
		t.Errorf("Determinant() = %v, want = %v", got, want)
	}
}

func TestOrthogonal(t *testing.T) {
	v := V{-1, 1}
	u := V{1, 1}
	want := true
	if got := IsOrthogonal(v, u); got != want {
		t.Errorf("IsOrthogonal() = %v, want = %v", got, want)
	}
}

func TestSquaredMagnitude(t *testing.T) {
	v := V{2, 3}
	want := float64(13)
	if got := SquaredMagnitude(v); got != want {
		t.Errorf("SquaredMagnitude() = %v, want = %v", got, want)
	}
}

func TestUnit(t *testing.T) {
	v := V{100, 0}
	want := V{1, 0}
	if got := Unit(v); got != want {
		t.Errorf("Unit() = %v, want = %v", got, want)
	}
}

func TestRotate(t *testing.T) {
	const tolerance = 1e-10
	testConfigs := []struct {
		name  string
		theta float64
		v     V
		want  V
	}{
		{name: "0Degree", theta: 0, v: V{1, 0}, want: V{1, 0}},
		{name: "90Degree", theta: .5 * math.Pi, v: V{1, 0}, want: V{0, 1}},
		{name: "180Degree", theta: math.Pi, v: V{1, 0}, want: V{-1, 0}},
		{name: "270Degree", theta: 1.5 * math.Pi, v: V{1, 0}, want: V{0, -1}},
		{name: "360Degree", theta: 2 * math.Pi, v: V{1, 0}, want: V{1, 0}},
		{name: "InverseRotate", theta: .1, v: Rotate(-.1, V{1, 0}), want: V{1, 0}},
		{
			name:  "FlipYCoordinate",
			theta: .2,
			v:     Rotate(-.1, V{1, 0}),
			want: *New(
				Rotate(-.1, V{1, 0}).X(),
				-Rotate(-.1, V{1, 0}).Y(),
			),
		},
		{
			name:  "FlipXCoordinate",
			theta: math.Pi + .2,
			v:     Rotate(-.1, V{1, 0}),
			want: *New(
				-Rotate(-.1, V{1, 0}).X(),
				Rotate(-.1, V{1, 0}).Y(),
			),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := Rotate(c.theta, c.v); !Within(got, c.want, tolerance) {
				t.Errorf("Rotate() = %v, want = %v", got, c.want)
			}
		})
	}
}
