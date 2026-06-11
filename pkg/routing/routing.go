package routing

import (
	"container/heap"
	"math"

	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/topology"
)

const (
	HugeDistanceMs     = 1e15
	HopsPerLinkPenalty = 1.0
)

type HeapNode struct {
	SatIndex   int
	F          float64
	G          float64
	Hops       int
	InsertSeq  uint64
}

type PriorityQueue struct {
	items []HeapNode
}

func (pq PriorityQueue) Len() int { return len(pq.items) }

func (pq PriorityQueue) Less(i, j int) bool {
	if pq.items[i].F != pq.items[j].F {
		return pq.items[i].F < pq.items[j].F
	}
	return pq.items[i].InsertSeq < pq.items[j].InsertSeq
}

func (pq PriorityQueue) Swap(i, j int) { pq.items[i], pq.items[j] = pq.items[j], pq.items[i] }

func (pq *PriorityQueue) Push(x interface{}) {
	pq.items = append(pq.items, x.(HeapNode))
}

func (pq *PriorityQueue) Pop() interface{} {
	old := pq.items
	n := len(old)
	item := old[n-1]
	pq.items = old[0 : n-1]
	return item
}

type RouteResult struct {
	Found               bool
	PathSatIndices      []int
	PathSatIDs          []uint64
	TotalHops           int
	TotalDistanceKm     float64
	TotalPropagationDelayMs float64
	LinkDistancesKm     []float64
	LinkDelaysMs        []float64
}

type Router struct {
	g             *topology.Graph
	distPool      *distSlicePool
	parentPool    *parentSlicePool
	visitedPool   *visitedSlicePool
}

type distSlicePool struct {
	pool [][]float64
	size int
}

func newDistSlicePool(size int) *distSlicePool {
	return &distSlicePool{
		pool: make([][]float64, 0, 1024),
		size: size,
	}
}

func (p *distSlicePool) get() []float64 {
	if len(p.pool) == 0 {
		s := make([]float64, p.size)
		for i := range s {
			s[i] = HugeDistanceMs
		}
		return s
	}
	s := p.pool[len(p.pool)-1]
	p.pool = p.pool[:len(p.pool)-1]
	for i := range s {
		s[i] = HugeDistanceMs
	}
	return s
}

func (p *distSlicePool) put(s []float64) {
	if len(p.pool) < 2048 {
		p.pool = append(p.pool, s)
	}
}

type parentSlicePool struct {
	pool [][]int
	size int
}

func newParentSlicePool(size int) *parentSlicePool {
	return &parentSlicePool{
		pool: make([][]int, 0, 1024),
		size: size,
	}
}

func (p *parentSlicePool) get() []int {
	var s []int
	if len(p.pool) == 0 {
		s = make([]int, p.size)
	} else {
		s = p.pool[len(p.pool)-1]
		p.pool = p.pool[:len(p.pool)-1]
	}
	for i := range s {
		s[i] = -1
	}
	return s
}

func (p *parentSlicePool) put(s []int) {
	if len(p.pool) < 2048 {
		p.pool = append(p.pool, s)
	}
}

type visitedSlicePool struct {
	pool [][]uint8
	size int
}

func newVisitedSlicePool(size int) *visitedSlicePool {
	return &visitedSlicePool{
		pool: make([][]uint8, 0, 1024),
		size: size,
	}
}

func (p *visitedSlicePool) get() []uint8 {
	var s []uint8
	if len(p.pool) == 0 {
		s = make([]uint8, p.size)
	} else {
		s = p.pool[len(p.pool)-1]
		p.pool = p.pool[:len(p.pool)-1]
	}
	for i := range s {
		s[i] = 0
	}
	return s
}

func (p *visitedSlicePool) put(s []uint8) {
	if len(p.pool) < 2048 {
		p.pool = append(p.pool, s)
	}
}

func NewRouter(g *topology.Graph) *Router {
	return &Router{
		g:           g,
		distPool:    newDistSlicePool(g.SatCount),
		parentPool:  newParentSlicePool(g.SatCount),
		visitedPool: newVisitedSlicePool(g.SatCount),
	}
}

func (r *Router) SetGraph(g *topology.Graph) {
	r.g = g
}

func heuristicEuclideanMs(positions []ephemeris.ECEFPosition, from, to int) float64 {
	dx := positions[from].XKm - positions[to].XKm
	dy := positions[from].YKm - positions[to].YKm
	dz := positions[from].ZKm - positions[to].ZKm
	distKm := math.Sqrt(dx*dx + dy*dy + dz*dz)
	return (distKm / 299792.458) * 1000.0
}

func (r *Router) computeAStar(srcIdx, dstIdx int, preferMinHops bool, alphaHopsMs float64) RouteResult {
	g := r.g
	n := g.SatCount

	if srcIdx < 0 || srcIdx >= n || dstIdx < 0 || dstIdx >= n {
		return RouteResult{Found: false}
	}
	if srcIdx == dstIdx {
		satID := g.SatIndexToID[srcIdx]
		return RouteResult{
			Found:                   true,
			PathSatIndices:          []int{srcIdx},
			PathSatIDs:              []uint64{satID},
			TotalHops:               0,
			TotalDistanceKm:         0,
			TotalPropagationDelayMs: 0,
		}
	}

	dist := r.distPool.get()
	defer r.distPool.put(dist)
	parent := r.parentPool.get()
	defer r.parentPool.put(parent)
	visited := r.visitedPool.get()
	defer r.visitedPool.put(visited)

	pq := PriorityQueue{items: make([]HeapNode, 0, n/10)}
	heap.Init(&pq)

	seq := uint64(0)
	dist[srcIdx] = 0
	h0 := heuristicEuclideanMs(g.Positions, srcIdx, dstIdx)
	heap.Push(&pq, HeapNode{
		SatIndex:  srcIdx,
		F:         h0,
		G:         0,
		Hops:      0,
		InsertSeq: seq,
	})
	seq++

	positions := g.Positions
	adjList := g.AdjList
	links := g.Links

	for pq.Len() > 0 {
		node := heap.Pop(&pq).(HeapNode)
		u := node.SatIndex

		if visited[u] != 0 {
			continue
		}
		visited[u] = 1

		if u == dstIdx {
			break
		}

		currentG := node.G
		currentHops := node.Hops

		if currentG > dist[u]+1e-9 {
			continue
		}

		for _, li := range adjList[u] {
			edge := &links[li]
			v := edge.ToSatIndex
			if visited[v] != 0 {
				continue
			}
			weightMs := edge.PropagationDelayMs + alphaHopsMs
			newG := currentG + weightMs
			if newG < dist[v] {
				dist[v] = newG
				parent[v] = u
				h := heuristicEuclideanMs(positions, v, dstIdx)
				heap.Push(&pq, HeapNode{
					SatIndex:  v,
					F:         newG + h,
					G:         newG,
					Hops:      currentHops + 1,
					InsertSeq: seq,
				})
				seq++
			}
		}
	}

	if dist[dstIdx] >= HugeDistanceMs-1 {
		return RouteResult{Found: false}
	}

	return r.reconstructPath(srcIdx, dstIdx, parent, alphaHopsMs)
}

func (r *Router) computeDijkstra(srcIdx, dstIdx int, preferMinHops bool, alphaHopsMs float64) RouteResult {
	g := r.g
	n := g.SatCount

	if srcIdx < 0 || srcIdx >= n || dstIdx < 0 || dstIdx >= n {
		return RouteResult{Found: false}
	}
	if srcIdx == dstIdx {
		satID := g.SatIndexToID[srcIdx]
		return RouteResult{
			Found:                   true,
			PathSatIndices:          []int{srcIdx},
			PathSatIDs:              []uint64{satID},
			TotalHops:               0,
			TotalDistanceKm:         0,
			TotalPropagationDelayMs: 0,
		}
	}

	dist := r.distPool.get()
	defer r.distPool.put(dist)
	parent := r.parentPool.get()
	defer r.parentPool.put(parent)
	visited := r.visitedPool.get()
	defer r.visitedPool.put(visited)

	pq := PriorityQueue{items: make([]HeapNode, 0, n/10)}
	heap.Init(&pq)

	seq := uint64(0)
	dist[srcIdx] = 0
	heap.Push(&pq, HeapNode{
		SatIndex:  srcIdx,
		F:         0,
		G:         0,
		Hops:      0,
		InsertSeq: seq,
	})
	seq++

	adjList := g.AdjList
	links := g.Links

	for pq.Len() > 0 {
		node := heap.Pop(&pq).(HeapNode)
		u := node.SatIndex

		if visited[u] != 0 {
			continue
		}
		visited[u] = 1

		if u == dstIdx {
			break
		}

		currentG := node.G
		if currentG > dist[u]+1e-9 {
			continue
		}

		for _, li := range adjList[u] {
			edge := &links[li]
			v := edge.ToSatIndex
			if visited[v] != 0 {
				continue
			}
			weightMs := edge.PropagationDelayMs + alphaHopsMs
			newG := currentG + weightMs
			if newG < dist[v] {
				dist[v] = newG
				parent[v] = u
				heap.Push(&pq, HeapNode{
					SatIndex:  v,
					F:         newG,
					G:         newG,
					Hops:      node.Hops + 1,
					InsertSeq: seq,
				})
				seq++
			}
		}
	}

	if dist[dstIdx] >= HugeDistanceMs-1 {
		return RouteResult{Found: false}
	}

	return r.reconstructPath(srcIdx, dstIdx, parent, alphaHopsMs)
}

func (r *Router) reconstructPath(srcIdx, dstIdx int, parent []int, alphaHopsMs float64) RouteResult {
	g := r.g
	path := make([]int, 0, 64)
	cur := dstIdx
	for cur != -1 {
		path = append(path, cur)
		if cur == srcIdx {
			break
		}
		cur = parent[cur]
	}

	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	result := RouteResult{
		Found:          true,
		PathSatIndices: path,
		PathSatIDs:     make([]uint64, len(path)),
		TotalHops:      len(path) - 1,
	}

	linkDistances := make([]float64, len(path))
	linkDelays := make([]float64, len(path))

	for i := 0; i < len(path); i++ {
		result.PathSatIDs[i] = g.SatIndexToID[path[i]]
	}

	adjList := g.AdjList
	links := g.Links
	positions := g.Positions

	for i := 1; i < len(path); i++ {
		prev := path[i-1]
		curr := path[i]
		found := false
		for _, li := range adjList[prev] {
			if links[li].ToSatIndex == curr {
				linkDistances[i] = links[li].DistanceKm
				linkDelays[i] = links[li].PropagationDelayMs
				result.TotalDistanceKm += links[li].DistanceKm
				result.TotalPropagationDelayMs += links[li].PropagationDelayMs
				found = true
				break
			}
		}
		if !found {
			p1 := positions[prev]
			p2 := positions[curr]
			dx := p1.XKm - p2.XKm
			dy := p1.YKm - p2.YKm
			dz := p1.ZKm - p2.ZKm
			distKm := math.Sqrt(dx*dx + dy*dy + dz*dz)
			delayMs := (distKm / 299792.458) * 1000.0
			linkDistances[i] = distKm
			linkDelays[i] = delayMs
			result.TotalDistanceKm += distKm
			result.TotalPropagationDelayMs += delayMs
		}
	}
	_ = alphaHopsMs

	result.LinkDistancesKm = linkDistances
	result.LinkDelaysMs = linkDelays

	return result
}

func (r *Router) FindPath(srcID, dstID uint64, preferMinHops bool, useAStar bool) RouteResult {
	g := r.g
	srcIdx, ok1 := g.SatIDToIndex[srcID]
	dstIdx, ok2 := g.SatIDToIndex[dstID]
	if !ok1 || !ok2 {
		return RouteResult{Found: false}
	}

	alphaHopsMs := 0.0
	if preferMinHops {
		alphaHopsMs = HopsPerLinkPenalty
	}

	if useAStar {
		return r.computeAStar(srcIdx, dstIdx, preferMinHops, alphaHopsMs)
	}
	return r.computeDijkstra(srcIdx, dstIdx, preferMinHops, alphaHopsMs)
}
