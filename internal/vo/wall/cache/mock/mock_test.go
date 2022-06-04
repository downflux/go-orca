package mock

import (
	"testing"

	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo"
	"github.com/downflux/go-orca/internal/vo/wall/cache/domain"
)

var (
	_ vo.VO = VO{}
)

func TestDomain(t *testing.T) {
	testConfigs := []struct {
		name  string
		vo    VO
		agent agent.A
		tau   float64
		want  domain.D
	}{}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.vo.domain(c.agent, c.tau); got != c.want {
				t.Errorf("domain() = %v, want = %v", got, c.want)
			}
		})
	}
}
