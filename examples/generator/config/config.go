package config

import (
	"encoding/json"
	"log"

	"github.com/downflux/go-orca/examples/agent"
	"github.com/downflux/go-orca/examples/segment"
)

type C struct {
	Agents   []agent.O
	Segments []segment.O
}

func Marshal(c C) []byte {
	b, err := json.MarshalIndent(c, "", " ")
	if err != nil {
		log.Fatalf("cannot export config: %v", err)
	}
	return b
}

func Unmarshal(data []byte) C {
	var c C
	if err := json.Unmarshal(data, &c); err != nil {
		log.Fatalf("cannot import config: %v", err)
	}
	return c
}
