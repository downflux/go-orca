package vector

import (
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
