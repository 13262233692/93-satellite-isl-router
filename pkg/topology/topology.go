package topology

import (
	"math"
	"sort"
	"sync"

	"satellite-isl-router/pkg/ephemeris"
)

const (
	DefaultMaxLaserLinkKm     = 5000.0
	DefaultOcclusionMarginKm  = 50.0
	DefaultGridCellSizeKm     = 800.0
	DefaultMaxLinksPerSat     = 20
)

type DirectedLink struct {
	FromSatIndex       int
	ToSatIndex         int
	FromSatID          uint64
	ToSatID            uint64
	DistanceKm         float64
	PropagationDelayMs float64
}

type TopologyConfig struct {
	MaxLaserLinkKm    float64
	OcclusionMarginKm float64
	GridCellSizeKm    float64
	MaxLinksPerSat    int
}

func DefaultTopologyConfig() TopologyConfig {
	return TopologyConfig{
		MaxLaserLinkKm:    DefaultMaxLaserLinkKm,
		OcclusionMarginKm: DefaultOcclusionMarginKm,
		GridCellSizeKm:    DefaultGridCellSizeKm,
		MaxLinksPerSat:    DefaultMaxLinksPerSat,
	}
}

type Graph struct {
	SatCount     int
	Links        []DirectedLink
	AdjList      [][]int
	SatIDToIndex map[uint64]int
	SatIndexToID []uint64
	Positions    []ephemeris.ECEFPosition
	BuildTimeNs  int64
}

func (g *Graph) GetSatIndex(id uint64) (int, bool) {
	idx, ok := g.SatIDToIndex[id]
	return idx, ok
}

type TopologyBuilder struct {
	config TopologyConfig
}

func NewTopologyBuilder(config TopologyConfig) *TopologyBuilder {
	if config.MaxLaserLinkKm <= 0 {
		config = DefaultTopologyConfig()
	}
	return &TopologyBuilder{config: config}
}

type gridCell struct {
	x, y, z int
}

func (g gridCell) Key() uint64 {
	const off = 1 << 20
	return (uint64(g.x+off) << 42) | (uint64(g.y+off) << 21) | uint64(g.z+off)
}

func positionToCell(p ephemeris.ECEFPosition, cellSize float64) gridCell {
	return gridCell{
		x: int(math.Floor(p.XKm / cellSize)),
		y: int(math.Floor(p.YKm / cellSize)),
		z: int(math.Floor(p.ZKm / cellSize)),
	}
}

func IsLineBlockedByEarth(p1, p2 ephemeris.ECEFPosition, marginKm float64) bool {
	R := ephemeris.EarthRadiusKm + marginKm
	dx := p2.XKm - p1.XKm
	dy := p2.YKm - p1.YKm
	dz := p2.ZKm - p1.ZKm
	a := dx*dx + dy*dy + dz*dz
	if a < 1e-10 {
		r1 := math.Sqrt(p1.XKm*p1.XKm + p1.YKm*p1.YKm + p1.ZKm*p1.ZKm)
		return r1 < R
	}
	b := 2 * (p1.XKm*dx + p1.YKm*dy + p1.ZKm*dz)
	c := p1.XKm*p1.XKm + p1.YKm*p1.YKm + p1.ZKm*p1.ZKm - R*R
	disc := b*b - 4*a*c
	if disc < 0 {
		return false
	}
	sqrtDisc := math.Sqrt(disc)
	t1 := (-b - sqrtDisc) / (2 * a)
	t2 := (-b + sqrtDisc) / (2 * a)
	return (t1 <= 1 && t2 >= 0)
}

type undirectedPair struct {
	i1, i2 int
	distKm float64
}

func (tb *TopologyBuilder) Build(snap *ephemeris.PositionSnapshot, buildTimeNs int64) *Graph {
	satCount := len(snap.Positions)
	g := &Graph{
		SatCount:     satCount,
		SatIDToIndex: make(map[uint64]int, satCount),
		SatIndexToID: make([]uint64, satCount),
		Positions:    make([]ephemeris.ECEFPosition, satCount),
		BuildTimeNs:  buildTimeNs,
	}
	for id, idx := range snap.IDToIndex {
		g.SatIDToIndex[id] = idx
		g.SatIndexToID[idx] = id
		g.Positions[idx] = snap.Positions[idx]
	}

	cellSize := tb.config.GridCellSizeKm
	grid := make(map[uint64][]int, satCount/4)
	for i := 0; i < satCount; i++ {
		cell := positionToCell(snap.Positions[i], cellSize)
		k := cell.Key()
		grid[k] = append(grid[k], i)
	}

	maxDistSq := tb.config.MaxLaserLinkKm * tb.config.MaxLaserLinkKm

	var pairs []undirectedPair
	var pairsMu sync.Mutex

	processPair := func(i1, i2 int, p1, p2 ephemeris.ECEFPosition) {
		if i1 == i2 {
			return
		}
		if i1 > i2 {
			i1, i2 = i2, i1
			p1, p2 = p2, p1
		}
		dx := p1.XKm - p2.XKm
		dy := p1.YKm - p2.YKm
		dz := p1.ZKm - p2.ZKm
		distSq := dx*dx + dy*dy + dz*dz
		if distSq > maxDistSq {
			return
		}
		if IsLineBlockedByEarth(p1, p2, tb.config.OcclusionMarginKm) {
			return
		}
		distKm := math.Sqrt(distSq)
		pairsMu.Lock()
		pairs = append(pairs, undirectedPair{i1, i2, distKm})
		pairsMu.Unlock()
	}

	cells := make([]gridCell, 0, len(grid))
	cellSats := make(map[uint64][]int, len(grid))
	for k, v := range grid {
		kx := int((k >> 42) - (1 << 20))
		ky := int(((k >> 21) & ((1 << 21) - 1)) - (1 << 20))
		kz := int((k & ((1 << 21) - 1)) - (1 << 20))
		cells = append(cells, gridCell{x: kx, y: ky, z: kz})
		cellSats[k] = v
	}

	for _, c := range cells {
		baseK := c.Key()
		baseSats := cellSats[baseK]

		for a := 0; a < len(baseSats); a++ {
			for b := a + 1; b < len(baseSats); b++ {
				i1 := baseSats[a]
				i2 := baseSats[b]
				processPair(i1, i2, snap.Positions[i1], snap.Positions[i2])
			}
		}

		for dz := 0; dz <= 1; dz++ {
			dyStart := 0
			if dz == 0 {
				dyStart = -1
			}
			for dy := dyStart; dy <= 1; dy++ {
				dxStart := -1
				if dz == 0 && dy == 0 {
					dxStart = 1
				}
				for dx := dxStart; dx <= 1; dx++ {
					nc := gridCell{x: c.x + dx, y: c.y + dy, z: c.z + dz}
					nk := nc.Key()
					nSats, ok := cellSats[nk]
					if !ok {
						continue
					}
					for a := 0; a < len(baseSats); a++ {
						for b := 0; b < len(nSats); b++ {
							i1 := baseSats[a]
							i2 := nSats[b]
							processPair(i1, i2, snap.Positions[i1], snap.Positions[i2])
						}
					}
				}
			}
		}
	}

	perSatOutLinks := make([][]undirectedPair, satCount)
	for _, p := range pairs {
		perSatOutLinks[p.i1] = append(perSatOutLinks[p.i1], p)
		rev := undirectedPair{p.i2, p.i1, p.distKm}
		perSatOutLinks[p.i2] = append(perSatOutLinks[p.i2], rev)
	}

	totalDirected := 0
	for i := 0; i < satCount; i++ {
		outs := perSatOutLinks[i]
		sort.Slice(outs, func(a, b int) bool {
			return outs[a].distKm < outs[b].distKm
		})
		if len(outs) > tb.config.MaxLinksPerSat {
			outs = outs[:tb.config.MaxLinksPerSat]
		}
		perSatOutLinks[i] = outs
		totalDirected += len(outs)
	}

	g.Links = make([]DirectedLink, 0, totalDirected)
	g.AdjList = make([][]int, satCount)
	for i := 0; i < satCount; i++ {
		for _, p := range perSatOutLinks[i] {
			fromID := g.SatIndexToID[p.i1]
			toID := g.SatIndexToID[p.i2]
			delayMs := ephemeris.PropagationDelayMs(p.distKm)
			link := DirectedLink{
				FromSatIndex:       p.i1,
				ToSatIndex:         p.i2,
				FromSatID:          fromID,
				ToSatID:            toID,
				DistanceKm:         p.distKm,
				PropagationDelayMs: delayMs,
			}
			linkIdx := len(g.Links)
			g.Links = append(g.Links, link)
			g.AdjList[p.i1] = append(g.AdjList[p.i1], linkIdx)
		}
	}

	return g
}

func (g *Graph) TotalLinks() int {
	return len(g.Links)
}

func (g *Graph) AvgDegree() float64 {
	if g.SatCount == 0 {
		return 0
	}
	total := 0
	for _, l := range g.AdjList {
		total += len(l)
	}
	return float64(total) / float64(g.SatCount)
}
