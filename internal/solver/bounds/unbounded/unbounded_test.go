package unbounded

import (
	s2d "github.com/downflux/go-orca/internal/solver/2d"
	s3d "github.com/downflux/go-orca/internal/solver/3d"
)

var (
	_ s2d.M = M{}
	_ s3d.M = M{}
)
