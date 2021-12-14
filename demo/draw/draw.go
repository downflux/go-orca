// Package draw is a small helper library to help render some demo-specific
// figures. The specific details aren't important in this package.
package draw

import (
	"image"
	"image/color"
	"image/draw"

	"github.com/downflux/go-geometry/2d/vector"
)

func Circle(img draw.Image, v vector.V, r int, c color.Color) {
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

func Trail(img *image.Paletted, margin vector.V, trailbuf []vector.V, c color.Color) {
	// Draw historical agent paths.
	for _, p := range trailbuf {
		p := vector.Add(margin, p)
		img.Set(int(p.X()), int(p.Y()), c)
	}
}
