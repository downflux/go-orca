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
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/orca"
)

const (
	N = 250

	S = 20 // m / s
	R = 10
	H = 1000
	W = 1000

	// Simulate 100 frames.
	D   = 10.  // s
	TAU = 1e-2 // 1/10 s
)

var _ agent.A = &A{}

type A struct {
	p vector.V
	g vector.V
	v vector.V
}

func (a *A) P() vector.V { return a.p }
func (a *A) T() vector.V {
	v := vector.Sub(a.g, a.p)
	if epsilon.Within(vector.SquaredMagnitude(v), 0) {
		return *vector.New(0, 0)
	}
	return vector.Scale(a.S(), vector.Unit(v))
}
func (a *A) V() vector.V { return a.v }
func (a *A) R() float64  { return R }
func (a *A) S() float64  { return S }

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

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

func GenerateRandomPoints(n int) []agent.A {
	agents := make([]agent.A, 0, n)
	for i := 0; i < N; i++ {
		p := vector.Add(
			*vector.New(rn(-500, 500), rn(-500, 500)),
			*vector.New(W/2, H/2),
		)
		g := vector.Add(
			p,
			*vector.New(rn(-100, 100), rn(-100, 100)),
		)

		a := &A{
			p: p,
			g: g,
		}
		a.v = a.T()

		agents = append(agents, a)
	}
	return agents
}

func GenerateLineCollision() []agent.A {
	offset := *vector.New(W/2, H/2)
	ps := []vector.V{
		vector.Add(*vector.New(-15, 5), offset),
		vector.Add(*vector.New(15, 0), offset),
	}

	agents := []agent.A{
		/*
			&A{
				p: vector.Add(*vector.New(0, 15), offset),
				g: *vector.New(0, -50),
				v: *vector.New(0, -S),
			},
		*/
		&A{
			p: ps[0],
			g: vector.Add(ps[0], *vector.New(15, 5)),
			v: *vector.New(S, 0),
		},
		&A{
			p: ps[1],
			g: vector.Add(ps[1], *vector.New(-15, 0)),
			v: *vector.New(-S, 0),
		},
	}
	return agents
}

func main() {
	agents := GenerateLineCollision()
	// agents := GenerateRandomPoints(N)
	points := make([]point.P, 0, N)

	for _, a := range agents {
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
				color.RGBA{255, 0, 255, 255},
				color.RGBA{0, 0, 255, 255},
				color.RGBA{0, 255, 0, 255},
			},
		)
		for x := 0; x < W; x++ {
			for y := 0; y < H; y++ {
				img.Set(x, y, color.White)
			}
		}

		for _, m := range res {
			a := m.A.(*A)

			c := color.RGBA{0, 0, 0, 255}
			// DEBUG
			if len(m.CS) > 0 {
				c.B = 255
			}
			// Vector has changed because Step() detected an
			// oncoming collision. Visually indicate this by
			// flashing the circle red.
			if !vector.Within(a.V(), vector.V(m.V)) {
				c.R = 255
			}

			if vector.Within(a.T(), *vector.New(0, 0)) {
				c = color.RGBA{0, 255, 0, 255}
			}

			DrawCircle(img, a.P(), R, c)

			// Draw agent goals.
			DrawCircle(img, a.g, 2, color.RGBA{0, 255, 0, 255})

			// Draw agent vision radii.
			DrawCircle(img, a.p, int(4*a.R()), color.RGBA{0, 255, 0, 255})

			a.p = vector.Add(
				a.P(),
				vector.Scale(TAU, vector.V(m.V)),
			)
			a.v = vector.V(m.V)
		}

		images = append(images, img)
		delay = append(delay, 2)

		tr.Balance()
	}

	// Export animation as a gif.
	anim := &gif.GIF{
		Delay: delay,
		Image: images,
	}
	f, _ := os.Create("demo/output/animation.gif")
	gif.EncodeAll(f, anim)
}
