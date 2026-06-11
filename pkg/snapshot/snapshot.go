package snapshot

import (
	"sync"
	"sync/atomic"
	"time"

	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/routing"
	"satellite-isl-router/pkg/topology"
)

const DefaultRefreshIntervalSec = 10

type Snapshot struct {
	EpochUnixMs uint64
	Positions   *ephemeris.PositionSnapshot
	Graph       *topology.Graph
	Routers     []*routing.Router
	routerPool  sync.Pool
	routerIdx   uint64
	routerCount int
}

func (s *Snapshot) GetRouter() *routing.Router {
	idx := atomic.AddUint64(&s.routerIdx, 1) % uint64(s.routerCount)
	return s.Routers[idx]
}

type Manager struct {
	ephModel      *ephemeris.EphemerisModel
	topoBuilder   *topology.TopologyBuilder
	refreshSec    int

	mu            sync.RWMutex
	active        *Snapshot
	standby       *Snapshot

	stopCh        chan struct{}
	wg            sync.WaitGroup
	running       atomic.Bool

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
		RouterConcurrency:  512,
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
		cfg.RouterConcurrency = 64
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
	snap := &Snapshot{
		EpochUnixMs: epoch,
		Positions:   pos,
		Graph:       g,
		Routers:     routers,
		routerCount: m.routerConcurrency,
	}
	m.active = snap
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
	return &Snapshot{
		EpochUnixMs: epoch,
		Positions:   pos,
		Graph:       g,
		Routers:     routers,
		routerCount: m.routerConcurrency,
	}
}

func (m *Manager) swap() {
	newSnap := m.buildStandbySnapshot()
	m.mu.Lock()
	m.standby = m.active
	m.active = newSnap
	m.mu.Unlock()
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
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.active
}

func (m *Manager) EphemerisModel() *ephemeris.EphemerisModel {
	return m.ephModel
}

func (m *Manager) RefreshIntervalSec() int {
	return m.refreshSec
}
