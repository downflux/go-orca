// Package main defines a small CLI app to generate pre-defined agent layouts.
//
// The app will print out the layouts in JSON format to stdout. This can be
// directly fed into the demo app, or saved in a file for later.
//
// Example:
//
//   go run main.go > config.json
package main

import (
	"flag"
	"fmt"

	"github.com/downflux/go-orca/demo/generator/generator"

	demo "github.com/downflux/go-orca/demo/agent"
)

const (
	N = 1000
	W = 1000
	H = 1000
	R = 10
	S = 50
)

type mode string

const (
	Random    mode = "random"
	Collision mode = "collision"
)

var (
	m = flag.String("mode", "random", "mode of the data generated, must be one of (random | collision)")
)

func main() {
	flag.Parse()

	fns := map[mode]func() []demo.O{
		Random:    func() []demo.O { return generator.R(W, H, S, R, N) },
		Collision: generator.C,
	}

	f, ok := fns[mode(*m)]
	if !ok {
		panic(fmt.Sprintf("invalid mode type %v", *m))
	}

	fmt.Printf(string(generator.Marshal(f())))
}
