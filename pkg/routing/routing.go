package routing

import (
	"container/heap"
	"math"
	"sync"

	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/topology"
)

const (
	HugeDistanceMs     = 1e15
	HopsPerLinkPenalty = 1.0
)

type HeapNode struct {
	SatIndex  int
	F         float64
	G         float64
	InsertSeq uint64
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
	Found                   bool
	PathSatIndices          []int
	PathSatIDs              []uint64
	TotalHops               int
	TotalDistanceKm         float64
	TotalPropagationDelayMs float64
	LinkDistancesKm         []float64
	LinkDelaysMs            []float64
}

var (
	distPool = sync.Pool{
		New: func() interface{} {
			return make([]float64, 0, 10240)
		},
	}
	parentPool = sync.Pool{
		New: func() interface{} {
			return make([]int, 0, 10240)
		},
	}
	visitedPool = sync.Pool{
		New: func() interface{} {
			return make([]uint8, 0, 10240)
		},
	}
	heapPool = sync.Pool{
		New: func() interface{} {
			return make([]HeapNode, 0, 2048)
		},
	}
	resultPool = sync.Pool{
		New: func() interface{} {
			return make([]int, 0, 128)
		},
	}
)

func getDistSlice(n int) []float64 {
	s := distPool.Get().([]float64)
	if cap(s) < n {
		distPool.Put(s)
		s = make([]float64, n)
	} else {
		s = s[:n]
	}
	for i := range s {
		s[i] = HugeDistanceMs
	}
	return s
}

func putDistSlice(s []float64) {
	distPool.Put(s)
}

func getParentSlice(n int) []int {
	s := parentPool.Get().([]int)
	if cap(s) < n {
		parentPool.Put(s)
		s = make([]int, n)
	} else {
		s = s[:n]
	}
	for i := range s {
		s[i] = -1
	}
	return s
}

func putParentSlice(s []int) {
	parentPool.Put(s)
}

func getVisitedSlice(n int) []uint8 {
	s := visitedPool.Get().([]uint8)
	if cap(s) < n {
		visitedPool.Put(s)
		s = make([]uint8, n)
	} else {
		s = s[:n]
	}
	for i := range s {
		s[i] = 0
	}
	return s
}

func putVisitedSlice(s []uint8) {
	visitedPool.Put(s)
}

type Router struct {
	g *topology.Graph
}

func NewRouter(g *topology.Graph) *Router {
	return &Router{g: g}
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

	dist := getDistSlice(n)
	defer putDistSlice(dist)
	parent := getParentSlice(n)
	defer putParentSlice(parent)
	visited := getVisitedSlice(n)
	defer putVisitedSlice(visited)

	heapStorage := heapPool.Get().([]HeapNode)
	pq := PriorityQueue{items: heapStorage[:0]}
	heap.Init(&pq)

	seq := uint64(0)
	dist[srcIdx] = 0
	h0 := heuristicEuclideanMs(g.Positions, srcIdx, dstIdx)
	heap.Push(&pq, HeapNode{
		SatIndex:  srcIdx,
		F:         h0,
		G:         0,
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
					InsertSeq: seq,
				})
				seq++
			}
		}
	}

	heapPool.Put(pq.items[:0])

	if dist[dstIdx] >= HugeDistanceMs-1 {
		return RouteResult{Found: false}
	}

	return r.reconstructPath(srcIdx, dstIdx, parent)
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

	dist := getDistSlice(n)
	defer putDistSlice(dist)
	parent := getParentSlice(n)
	defer putParentSlice(parent)
	visited := getVisitedSlice(n)
	defer putVisitedSlice(visited)

	heapStorage := heapPool.Get().([]HeapNode)
	pq := PriorityQueue{items: heapStorage[:0]}
	heap.Init(&pq)

	seq := uint64(0)
	dist[srcIdx] = 0
	heap.Push(&pq, HeapNode{
		SatIndex:  srcIdx,
		F:         0,
		G:         0,
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
					InsertSeq: seq,
				})
				seq++
			}
		}
	}

	heapPool.Put(pq.items[:0])

	if dist[dstIdx] >= HugeDistanceMs-1 {
		return RouteResult{Found: false}
	}

	return r.reconstructPath(srcIdx, dstIdx, parent)
}

func (r *Router) reconstructPath(srcIdx, dstIdx int, parent []int) RouteResult {
	g := r.g

	pathBack := resultPool.Get().([]int)
	pathBack = pathBack[:0]

	cur := dstIdx
	depth := 0
	for cur != -1 && depth < 4096 {
		pathBack = append(pathBack, cur)
		if cur == srcIdx {
			break
		}
		cur = parent[cur]
		depth++
	}

	pathLen := len(pathBack)
	path := make([]int, pathLen)
	for i := 0; i < pathLen; i++ {
		path[i] = pathBack[pathLen-1-i]
	}
	resultPool.Put(pathBack[:0])

	result := RouteResult{
		Found:          true,
		PathSatIndices: path,
		PathSatIDs:     make([]uint64, pathLen),
		TotalHops:      pathLen - 1,
	}

	linkDistances := make([]float64, pathLen)
	linkDelays := make([]float64, pathLen)

	for i := 0; i < pathLen; i++ {
		result.PathSatIDs[i] = g.SatIndexToID[path[i]]
	}

	adjList := g.AdjList
	links := g.Links
	positions := g.Positions

	for i := 1; i < pathLen; i++ {
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
