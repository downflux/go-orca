// Package main executes a short demo of ORCA and outputs a gif of the
// calculated trajectory of the set of agents.
//
// Run with
//   go run github.com/downflux/go-orca/demo
package main

import (
	"image"
	"image/color"
	"image/draw"
	"image/gif"
	"math/rand"
	"os"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/orca"
)

var _ agent.A = &A{}

type A struct {
	p vector.V
	t vector.V
	v vector.V
}

func (a *A) P() vector.V { return a.p }
func (a *A) T() vector.V { return a.t }
func (a *A) V() vector.V { return a.v }
func (a *A) R() float64  { return R }
func (a *A) S() float64  { return S }

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

const (
	N = 250

	S = 100
	R = 5
	H = 1000
	W = 1000

	// Simulate 100 frames.
	D   = 1.
	TAU = 1e-2
)

func DrawCircle(img draw.Image, v vector.V, r int, c color.Color) {
	x, y, dx, dy := r-1, 0, 1, 1
	err := dx - (r * 2)

	for x > y {
		img.Set(int(v.X())+x, int(v.Y())+y, c)
		img.Set(int(v.X())+y, int(v.Y())+x, c)
		img.Set(int(v.X())-y, int(v.Y())+x, c)
		img.Set(int(v.X())-x, int(v.Y())+y, c)
		img.Set(int(v.X())-x, int(v.Y())-y, c)
		img.Set(int(v.X())-y, int(v.Y())-x, c)
		img.Set(int(v.X())+y, int(v.Y())-x, c)
		img.Set(int(v.X())+x, int(v.Y())-y, c)

		if err <= 0 {
			y++
			err += dy
			dy += 2
		}
		if err > 0 {
			x--
			dx += 2
			err += dx - (r * 2)
		}
	}
}

func main() {
	agents := make([]agent.A, 0, N)
	points := make([]point.P, 0, N)
	for i := 0; i < N; i++ {
		p := *vector.New(rn(-500, 500), rn(-500, 500))
		t := *vector.New(rn(-500, 500), rn(-500, 500))
		v := vector.Scale(S, vector.Unit(t))

		a := &A{
			p: p,
			t: t,
			v: v,
		}

		agents = append(agents, a)
		points = append(points, *orca.New(a))
	}

	tr, err := kd.New(points)
	if err != nil {
		panic("cannot create K-D tree")
	}

	var images []*image.Paletted
	var delay []int

	for t := 0.; t < D; t += TAU {
		res, err := orca.Step(
			tr,
			TAU,
			func(a agent.A) bool { return true },
		)

		if err != nil {
			panic("error while stepping through ORCA")
		}

		img := image.NewPaletted(
			image.Rectangle{
				image.Point{0, 0},
				image.Point{W, H},
			},
			[]color.Color{
				color.White,
				color.Black,
				color.RGBA{255, 0, 0, 255},
			},
		)
		for x := 0; x < W; x++ {
			for y := 0; y < H; y++ {
				img.Set(x, y, color.White)
			}
		}

		for _, m := range res {
			a := m.A.(*A)

			var c color.Color = color.Black
			// Vector has changed because Step() detected an
			// oncoming collision. Visually indicate this by
			// flashing the circle red.
			if !vector.Within(a.v, vector.V(m.V)) {
				c = color.RGBA{255, 0, 0, 255}
			}

			// Center agents in image.
			p := vector.Add(
				vector.V(a.P()),
				*vector.New(W/2, H/2),
			)
			DrawCircle(img, p, R, c)

			a.p = vector.Add(
				a.P(),
				vector.Scale(TAU, vector.V(m.V)),
			)
			a.v = vector.V(m.V)
		}

		images = append(images, img)
		delay = append(delay, 0)

		tr.Balance()
	}

	// Export animation as a gif.
	anim := &gif.GIF{
		Delay: delay,
		Image: images,
	}
	f, _ := os.Create("output/animation.gif")
	gif.EncodeAll(f, anim)
}
