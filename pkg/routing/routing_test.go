package routing

import (
	"fmt"
	"math/rand"
	"testing"
	"time"

	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/topology"
)

func buildTestGraph(satCount int) *topology.Graph {
	ephCfg := ephemeris.DefaultConstellationConfig()
	ephCfg.TotalSatellites = satCount
	ephCfg.NumOrbitPlanes = 10
	ephCfg.SatsPerPlane = satCount / 10
	if ephCfg.SatsPerPlane*ephCfg.NumOrbitPlanes < satCount {
		ephCfg.SatsPerPlane++
	}
	ephCfg.RAANDeltaDeg = 180.0 / float64(ephCfg.NumOrbitPlanes)
	eph := ephemeris.NewEphemerisModel(ephCfg)
	epoch := uint64(time.Now().UnixMilli())
	pos := eph.ComputeAllPositions(epoch)
	tb := topology.NewTopologyBuilder(topology.DefaultTopologyConfig())
	return tb.Build(pos, 0)
}

func TestDijkstraSmall(t *testing.T) {
	g := buildTestGraph(500)
	t.Logf("Graph: %d sats, %d links, avg deg %.2f", g.SatCount, g.TotalLinks(), g.AvgDegree())
	r := NewRouter(g)

	ids := make([]uint64, g.SatCount)
	for i := 0; i < g.SatCount; i++ {
		ids[i] = g.SatIndexToID[i]
	}

	rng := rand.New(rand.NewSource(7))
	for i := 0; i < 20; i++ {
		a := rng.Intn(g.SatCount)
		b := rng.Intn(g.SatCount)
		for a == b {
			b = rng.Intn(g.SatCount)
		}
		srcID := ids[a]
		dstID := ids[b]

		start := time.Now()
		res := r.FindPath(srcID, dstID, false, false)
		elapsed := time.Since(start)

		if !res.Found {
			t.Logf("Path %d->%d not found", srcID, dstID)
			continue
		}
		if res.PathSatIDs[0] != srcID || res.PathSatIDs[len(res.PathSatIDs)-1] != dstID {
			t.Errorf("Path endpoints mismatch: %d->%d != %d->%d",
				res.PathSatIDs[0], res.PathSatIDs[len(res.PathSatIDs)-1], srcID, dstID)
		}
		if res.TotalHops != len(res.PathSatIDs)-1 {
			t.Errorf("Hop count mismatch: %d vs %d", res.TotalHops, len(res.PathSatIDs)-1)
		}
		if elapsed.Microseconds() > 2000 {
			t.Logf("Slow path %d->%d: %d us, %d hops", srcID, dstID, elapsed.Microseconds(), res.TotalHops)
		}
		t.Logf("Path %d->%d: %d hops, %.1f km, %.3f ms, %d us",
			srcID, dstID, res.TotalHops, res.TotalDistanceKm, res.TotalPropagationDelayMs, elapsed.Microseconds())
	}
}

func TestAStarVsDijkstra(t *testing.T) {
	g := buildTestGraph(2000)
	r := NewRouter(g)
	ids := make([]uint64, g.SatCount)
	for i := 0; i < g.SatCount; i++ {
		ids[i] = g.SatIndexToID[i]
	}
	rng := rand.New(rand.NewSource(99))
	tests := 50
	matchCount := 0
	for i := 0; i < tests; i++ {
		a := rng.Intn(g.SatCount)
		b := rng.Intn(g.SatCount)
		if a == b {
			continue
		}
		resD := r.FindPath(ids[a], ids[b], false, false)
		resA := r.FindPath(ids[a], ids[b], false, true)
		if resD.Found != resA.Found {
			t.Errorf("Dijkstra found=%v A* found=%v for %d->%d", resD.Found, resA.Found, ids[a], ids[b])
			continue
		}
		if !resD.Found {
			continue
		}
		if resD.TotalHops != resA.TotalHops {
			t.Logf("Hop diff Dijk=%d A*=%d for %d->%d D=%.0fkm vs %.0fkm",
				resD.TotalHops, resA.TotalHops, ids[a], ids[b],
				resD.TotalDistanceKm, resA.TotalDistanceKm)
		}
		if mathAbsDiff(resD.TotalPropagationDelayMs, resA.TotalPropagationDelayMs) < 0.1 {
			matchCount++
		}
	}
	t.Logf("A* vs Dijkstra delay-matched %d/%d", matchCount, tests)
}

func mathAbsDiff(a, b float64) float64 {
	if a > b {
		return a - b
	}
	return b - a
}

func BenchmarkDijkstra10k(b *testing.B) {
	g := buildTestGraph(10000)
	fmt.Printf("Bench graph: %d sats, %d links\n", g.SatCount, g.TotalLinks())
	r := NewRouter(g)
	ids := make([]uint64, g.SatCount)
	for i := 0; i < g.SatCount; i++ {
		ids[i] = g.SatIndexToID[i]
	}
	rng := rand.New(rand.NewSource(42))
	pairs := make([][2]uint64, b.N)
	for i := 0; i < b.N; i++ {
		a := rng.Intn(g.SatCount)
		c := rng.Intn(g.SatCount)
		for a == c {
			c = rng.Intn(g.SatCount)
		}
		pairs[i] = [2]uint64{ids[a], ids[c]}
	}
	b.ResetTimer()
	over1ms := 0
	for i := 0; i < b.N; i++ {
		start := time.Now()
		res := r.FindPath(pairs[i][0], pairs[i][1], false, false)
		elapsed := time.Since(start)
		if elapsed.Nanoseconds() > 1e6 {
			over1ms++
		}
		_ = res
	}
	b.Logf("Over 1ms: %d/%d (%.1f%%)", over1ms, b.N, 100.0*float64(over1ms)/float64(b.N))
}

func BenchmarkAStar10k(b *testing.B) {
	g := buildTestGraph(10000)
	r := NewRouter(g)
	ids := make([]uint64, g.SatCount)
	for i := 0; i < g.SatCount; i++ {
		ids[i] = g.SatIndexToID[i]
	}
	rng := rand.New(rand.NewSource(42))
	pairs := make([][2]uint64, b.N)
	for i := 0; i < b.N; i++ {
		a := rng.Intn(g.SatCount)
		c := rng.Intn(g.SatCount)
		for a == c {
			c = rng.Intn(g.SatCount)
		}
		pairs[i] = [2]uint64{ids[a], ids[c]}
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		res := r.FindPath(pairs[i][0], pairs[i][1], false, true)
		_ = res
	}
}
