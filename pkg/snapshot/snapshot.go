package snapshot

import (
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/routing"
	"satellite-isl-router/pkg/topology"
)

const DefaultRefreshIntervalSec = 10

const (
	cacheLinePadSize = 64
	defaultRouterShards = 128
)

type routerShard struct {
	randState uint64
	_padding0 [7]uint64
	_padding1 [cacheLinePadSize - 64]byte
}

type Snapshot struct {
	EpochUnixMs uint64
	Positions   *ephemeris.PositionSnapshot
	Graph       *topology.Graph
	Routers     []*routing.Router
	routerCount int
	routerMask  int

	shards     []routerShard
	shardCount int
	shardMask  int
	_padding2  [cacheLinePadSize]byte
}

func newSnapshotInternal(
	epoch uint64,
	pos *ephemeris.PositionSnapshot,
	g *topology.Graph,
	routers []*routing.Router,
	routerCount int,
) *Snapshot {
	rc := 1
	for rc < routerCount {
		rc <<= 1
	}
	realRouters := routers
	if rc != routerCount {
		rc = routerCount
	}

	shardCount := 1
	wantShards := runtime.GOMAXPROCS(0) * 8
	if wantShards < defaultRouterShards {
		wantShards = defaultRouterShards
	}
	for shardCount < wantShards {
		shardCount <<= 1
	}
	shards := make([]routerShard, shardCount)
	seedBase := uint64(time.Now().UnixNano())
	const goldenRatio = uint64(0x9E3779B97F4A7C15)
	for i := range shards {
		shards[i].randState = seedBase ^ (uint64(i) * goldenRatio) + 1
	}

	routerMask := rc - 1
	if rc&(rc-1) != 0 {
		routerMask = 0
	}

	return &Snapshot{
		EpochUnixMs: epoch,
		Positions:   pos,
		Graph:       g,
		Routers:     realRouters,
		routerCount: rc,
		routerMask:  routerMask,
		shards:      shards,
		shardCount:  shardCount,
		shardMask:   shardCount - 1,
	}
}

func xorshift64starLocal(state uint64) (uint64, uint64) {
	if state == 0 {
		state = 0x123456789ABCDEF0
	}
	x := state
	x ^= x >> 12
	x ^= x << 25
	x ^= x >> 27
	return x * 0x2545F4914F6CDD1D, x
}

func (s *Snapshot) GetRouter() *routing.Router {
	n := s.routerCount
	if n == 1 {
		return s.Routers[0]
	}
	shard := &s.shards[uint64(time.Now().UnixNano())&uint64(s.shardMask)]
	cur := atomic.LoadUint64(&shard.randState)
	rnd, next := xorshift64starLocal(cur)
	atomic.CompareAndSwapUint64(&shard.randState, cur, next)

	if s.routerMask != 0 {
		return s.Routers[rnd&uint64(s.routerMask)]
	}
	return s.Routers[rnd%uint64(n)]
}

func (s *Snapshot) GetRouterByToken(token uint64) *routing.Router {
	n := s.routerCount
	if n == 1 {
		return s.Routers[0]
	}
	if s.routerMask != 0 {
		return s.Routers[token&uint64(s.routerMask)]
	}
	return s.Routers[token%uint64(n)]
}

type Manager struct {
	ephModel    *ephemeris.EphemerisModel
	topoBuilder *topology.TopologyBuilder
	refreshSec  int

	active atomic.Value

	stopCh  chan struct{}
	wg      sync.WaitGroup
	running atomic.Bool

	routerConcurrency int
}

type ManagerConfig struct {
	RefreshIntervalSec int
	RouterConcurrency  int
	EphemerisConfig    ephemeris.ConstellationConfig
	TopologyConfig     topology.TopologyConfig
}

func DefaultManagerConfig() ManagerConfig {
	cfg := ManagerConfig{
		RefreshIntervalSec: DefaultRefreshIntervalSec,
		RouterConcurrency:  1024,
		EphemerisConfig:    ephemeris.DefaultConstellationConfig(),
		TopologyConfig:     topology.DefaultTopologyConfig(),
	}
	return cfg
}

func NewManager(cfg ManagerConfig) *Manager {
	if cfg.RefreshIntervalSec <= 0 {
		cfg.RefreshIntervalSec = DefaultRefreshIntervalSec
	}
	if cfg.RouterConcurrency <= 0 {
		cfg.RouterConcurrency = 512
	}
	eph := ephemeris.NewEphemerisModel(cfg.EphemerisConfig)
	tb := topology.NewTopologyBuilder(cfg.TopologyConfig)
	m := &Manager{
		ephModel:          eph,
		topoBuilder:       tb,
		refreshSec:        cfg.RefreshIntervalSec,
		stopCh:            make(chan struct{}),
		routerConcurrency: cfg.RouterConcurrency,
	}
	m.buildInitialSnapshot()
	return m
}

func (m *Manager) buildInitialSnapshot() {
	epoch := uint64(time.Now().UnixMilli())
	startBuild := time.Now()
	pos := m.ephModel.ComputeAllPositions(epoch)
	buildNs := time.Since(startBuild).Nanoseconds()
	g := m.topoBuilder.Build(pos, buildNs)
	routers := make([]*routing.Router, m.routerConcurrency)
	for i := 0; i < m.routerConcurrency; i++ {
		routers[i] = routing.NewRouter(g)
	}
	snap := newSnapshotInternal(epoch, pos, g, routers, m.routerConcurrency)
	m.active.Store(snap)
}

func (m *Manager) buildStandbySnapshot() *Snapshot {
	epoch := uint64(time.Now().UnixMilli())
	startBuild := time.Now()
	pos := m.ephModel.ComputeAllPositions(epoch)
	buildNs := time.Since(startBuild).Nanoseconds()
	g := m.topoBuilder.Build(pos, buildNs)
	routers := make([]*routing.Router, m.routerConcurrency)
	for i := 0; i < m.routerConcurrency; i++ {
		routers[i] = routing.NewRouter(g)
	}
	return newSnapshotInternal(epoch, pos, g, routers, m.routerConcurrency)
}

func (m *Manager) swap() {
	newSnap := m.buildStandbySnapshot()
	m.active.Store(newSnap)
}

func (m *Manager) Swap() {
	m.swap()
}

func (m *Manager) Start() {
	if !m.running.CompareAndSwap(false, true) {
		return
	}
	m.wg.Add(1)
	go m.loop()
}

func (m *Manager) Stop() {
	if !m.running.CompareAndSwap(true, false) {
		return
	}
	close(m.stopCh)
	m.wg.Wait()
}

func (m *Manager) loop() {
	defer m.wg.Done()
	ticker := time.NewTicker(time.Duration(m.refreshSec) * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ticker.C:
			m.swap()
		case <-m.stopCh:
			return
		}
	}
}

func (m *Manager) Active() *Snapshot {
	v := m.active.Load()
	if v == nil {
		return nil
	}
	return v.(*Snapshot)
}

func (m *Manager) EphemerisModel() *ephemeris.EphemerisModel {
	return m.ephModel
}

func (m *Manager) RefreshIntervalSec() int {
	return m.refreshSec
}
