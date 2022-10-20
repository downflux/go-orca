// Package main defines a small CLI app to generate pre-defined agent layouts.
//
// The app will print out the layouts in JSON format to stdout. This can be
// directly fed into the demo app, or saved in a file for later.
//
// Example:
//
//	go run main.go > config.json
package main

import (
	"flag"
	"fmt"
	"log"

	"github.com/downflux/go-orca/examples/config"
	"github.com/downflux/go-orca/examples/generator/generator"
)

const (
	N = 250
	W = 1000
	H = 1000
	R = 10
	S = 55

	X = 10
	Y = 10
)

type mode string

const (
	Random      mode = "random"
	Collision   mode = "collision"
	Grid        mode = "grid"
	Line        mode = "line"
	DebugCanvas mode = "debug"
)

var (
	m = flag.String("mode", "random", "mode of the data generated, must be one of (random | collision | grid)")
)

func main() {
	flag.Parse()

	fns := map[mode]func() config.O{
		Random:      func() config.O { return generator.R(W, H, S, R, N) },
		Collision:   func() config.O { return generator.C(S, R) },
		Grid:        func() config.O { return generator.G(X, Y, S, R) },
		Line:        func() config.O { return generator.L(X, Y, S, R) },
		DebugCanvas: func() config.O { return generator.DebugCanvas() },
	}

	f, ok := fns[mode(*m)]
	if !ok {
		log.Fatalf("invalid mode type %v", *m)
	}

	fmt.Printf(string(config.Marshal(f())))
}
