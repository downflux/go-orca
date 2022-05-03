package config

import (
	"encoding/json"
	"log"

	"github.com/downflux/go-orca/examples/agent"
	"github.com/downflux/go-orca/examples/segment"
)

type O struct {
	Agents   []agent.O
	Segments []segment.O
}

func Marshal(o O) []byte {
	b, err := json.MarshalIndent(o, "", " ")
	if err != nil {
		log.Fatalf("cannot export config: %v", err)
	}
	return b
}

func Unmarshal(data []byte) O {
	var o O
	if err := json.Unmarshal(data, &o); err != nil {
		log.Fatalf("cannot import config: %v", err)
	}
	return o
}
