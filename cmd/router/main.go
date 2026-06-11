package main

import (
	"flag"
	"fmt"
	"log"
	"math/rand"
	"net"
	"os"
	"os/signal"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	pb "satellite-isl-router/proto"
	"satellite-isl-router/pkg/ephemeris"
	"satellite-isl-router/pkg/server"
	"satellite-isl-router/pkg/snapshot"
	"satellite-isl-router/pkg/topology"

	grpc "google.golang.org/grpc"
	"google.golang.org/grpc/reflection"
)

func main() {
	port := flag.Int("port", 50051, "gRPC server port")
	refreshSec := flag.Int("refresh", 10, "Topology refresh interval seconds")
	satCount := flag.Int("sats", 10000, "Total satellites")
	altitudeKm := flag.Float64("alt", 550.0, "Satellite altitude km")
	incDeg := flag.Float64("inc", 53.0, "Orbit inclination degrees")
	maxLinkKm := flag.Float64("maxlink", 5000.0, "Max laser link km")
	useAStar := flag.Bool("astar", true, "Use A* algorithm (false for Dijkstra)")
	routerConcurrency := flag.Int("conc", 512, "Router instances for concurrency")
	planes := flag.Int("planes", 100, "Number of orbit planes")
	satsPerPlane := flag.Int("spp", 100, "Satellites per plane")
	maxLinksPerSat := flag.Int("maxlinks", 20, "Max directed links per satellite")
	runBenchmark := flag.Bool("bench", false, "Run internal benchmark and exit")

	flag.Parse()

	ephCfg := ephemeris.DefaultConstellationConfig()
	ephCfg.TotalSatellites = *satCount
	ephCfg.AltitudeKm = *altitudeKm
	ephCfg.InclinationDeg = *incDeg
	if *planes > 0 && *satsPerPlane > 0 {
		ephCfg.NumOrbitPlanes = *planes
		ephCfg.SatsPerPlane = *satsPerPlane
		ephCfg.RAANDeltaDeg = 180.0 / float64(*planes)
	}

	topoCfg := topology.DefaultTopologyConfig()
	topoCfg.MaxLaserLinkKm = *maxLinkKm
	topoCfg.MaxLinksPerSat = *maxLinksPerSat

	mgrCfg := snapshot.DefaultManagerConfig()
	mgrCfg.RefreshIntervalSec = *refreshSec
	mgrCfg.EphemerisConfig = ephCfg
	mgrCfg.TopologyConfig = topoCfg
	mgrCfg.RouterConcurrency = *routerConcurrency

	fmt.Printf("Initializing constellation: %d satellites, %d planes, %d sats/plane, %.0fkm alt, %.1f° inc\n",
		ephCfg.TotalSatellites, ephCfg.NumOrbitPlanes, ephCfg.SatsPerPlane,
		ephCfg.AltitudeKm, ephCfg.InclinationDeg)

	snapMgr := snapshot.NewManager(mgrCfg)

	active := snapMgr.Active()
	fmt.Printf("Initial topology built: %d satellites, %d directed links, avg degree=%.2f\n",
		active.Graph.SatCount, active.Graph.TotalLinks(), active.Graph.AvgDegree())
	fmt.Printf("Topology build time: %.2f ms\n", float64(active.Graph.BuildTimeNs)/1e6)
	fmt.Printf("Refresh interval: %d s, Router concurrency: %d\n", *refreshSec, *routerConcurrency)

	if *runBenchmark {
		runBench(snapMgr, *useAStar)
		return
	}

	snapMgr.Start()
	defer snapMgr.Stop()

	lis, err := net.Listen("tcp", fmt.Sprintf(":%d", *port))
	if err != nil {
		log.Fatalf("failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer(
		grpc.NumStreamWorkers(uint32(*routerConcurrency)),
		grpc.MaxConcurrentStreams(uint32(*routerConcurrency)*16),
	)

	routerSrv := server.NewRouterGRPCServer(snapMgr, *useAStar)
	pb.RegisterRouterServiceServer(grpcServer, routerSrv)
	reflection.Register(grpcServer)

	stopCh := make(chan os.Signal, 1)
	signal.Notify(stopCh, syscall.SIGINT, syscall.SIGTERM)

	go func() {
		fmt.Printf("gRPC server listening on port %d\n", *port)
		if err := grpcServer.Serve(lis); err != nil {
			log.Fatalf("failed to serve: %v", err)
		}
	}()

	<-stopCh
	fmt.Println("Shutting down...")
	grpcServer.GracefulStop()
	fmt.Println("Server stopped")
}

func runBench(snapMgr *snapshot.Manager, useAStar bool) {
	active := snapMgr.Active()
	g := active.Graph
	n := g.SatCount
	ids := make([]uint64, n)
	for i := 0; i < n; i++ {
		ids[i] = g.SatIndexToID[i]
	}

	fmt.Println("\n======= Single Path Benchmark (Sequential) =======")
	rng := rand.New(rand.NewSource(42))
	totalPaths := 200
	successCount := 0
	var totalTimeNs int64 = 0
	maxTimeNs := int64(0)
	minTimeNs := int64(1 << 62)
	totalHops := 0
	totalDelayMs := 0.0
	totalDistKm := 0.0

	r := active.GetRouter()
	for i := 0; i < totalPaths; i++ {
		a := rng.Intn(n)
		b := rng.Intn(n)
		for a == b {
			b = rng.Intn(n)
		}
		srcID := ids[a]
		dstID := ids[b]

		start := time.Now()
		res := r.FindPath(srcID, dstID, false, useAStar)
		elapsed := time.Since(start).Nanoseconds()

		totalTimeNs += elapsed
		if elapsed > maxTimeNs {
			maxTimeNs = elapsed
		}
		if elapsed < minTimeNs {
			minTimeNs = elapsed
		}
		if res.Found {
			successCount++
			totalHops += res.TotalHops
			totalDelayMs += res.TotalPropagationDelayMs
			totalDistKm += res.TotalDistanceKm
		}
	}
	fmt.Printf("Algorithm: %s\n", map[bool]string{true: "A*", false: "Dijkstra"}[useAStar])
	fmt.Printf("Total paths: %d, Success: %d (%.1f%%)\n", totalPaths, successCount, 100.0*float64(successCount)/float64(totalPaths))
	fmt.Printf("Avg compute: %.2f us, Max: %.2f us, Min: %.2f us\n",
		float64(totalTimeNs)/float64(totalPaths)/1e3,
		float64(maxTimeNs)/1e3,
		float64(minTimeNs)/1e3)
	if successCount > 0 {
		fmt.Printf("Avg hops: %.1f, Avg delay: %.3f ms, Avg dist: %.0f km\n",
			float64(totalHops)/float64(successCount),
			totalDelayMs/float64(successCount),
			totalDistKm/float64(successCount))
	}

	fmt.Println("\n======= Single Path Benchmark (Dijkstra) =======")
	rng2 := rand.New(rand.NewSource(123))
	totalPaths2 := 200
	successCount2 := 0
	var totalTimeNs2 int64 = 0
	maxTimeNs2 := int64(0)
	minTimeNs2 := int64(1 << 62)
	totalHops2 := 0
	totalDelayMs2 := 0.0
	totalDistKm2 := 0.0

	for i := 0; i < totalPaths2; i++ {
		a := rng2.Intn(n)
		b := rng2.Intn(n)
		for a == b {
			b = rng2.Intn(n)
		}
		srcID := ids[a]
		dstID := ids[b]

		start := time.Now()
		res := r.FindPath(srcID, dstID, false, false)
		elapsed := time.Since(start).Nanoseconds()

		totalTimeNs2 += elapsed
		if elapsed > maxTimeNs2 {
			maxTimeNs2 = elapsed
		}
		if elapsed < minTimeNs2 {
			minTimeNs2 = elapsed
		}
		if res.Found {
			successCount2++
			totalHops2 += res.TotalHops
			totalDelayMs2 += res.TotalPropagationDelayMs
			totalDistKm2 += res.TotalDistanceKm
		}
	}
	fmt.Printf("Algorithm: Dijkstra\n")
	fmt.Printf("Total paths: %d, Success: %d (%.1f%%)\n", totalPaths2, successCount2, 100.0*float64(successCount2)/float64(totalPaths2))
	fmt.Printf("Avg compute: %.2f us, Max: %.2f us, Min: %.2f us\n",
		float64(totalTimeNs2)/float64(totalPaths2)/1e3,
		float64(maxTimeNs2)/1e3,
		float64(minTimeNs2)/1e3)
	if successCount2 > 0 {
		fmt.Printf("Avg hops: %.1f, Avg delay: %.3f ms, Avg dist: %.0f km\n",
			float64(totalHops2)/float64(successCount2),
			totalDelayMs2/float64(successCount2),
			totalDistKm2/float64(successCount2))
	}

	fmt.Println("\n======= Concurrent Path Benchmark (256 workers, A*) =======")
	concurrency := 256
	pathsPerWorker := 200
	var wg sync.WaitGroup
	var totalConcurrent int64 = 0
	var successConcurrent int64 = 0
	var totalConcurrentTimeNs int64 = 0
	var over1ms int64 = 0
	var over200us int64 = 0
	var over500us int64 = 0

	benchStart := time.Now()
	for w := 0; w < concurrency; w++ {
		wg.Add(1)
		go func(seed int64) {
			defer wg.Done()
			localRng := rand.New(rand.NewSource(seed))
			localR := active.GetRouter()
			for p := 0; p < pathsPerWorker; p++ {
				a := localRng.Intn(n)
				b := localRng.Intn(n)
				for a == b {
					b = localRng.Intn(n)
				}
				srcID := ids[a]
				dstID := ids[b]
				start := time.Now()
				res := localR.FindPath(srcID, dstID, false, useAStar)
				elapsed := time.Since(start).Nanoseconds()
				atomic.AddInt64(&totalConcurrentTimeNs, elapsed)
				atomic.AddInt64(&totalConcurrent, 1)
				if res.Found {
					atomic.AddInt64(&successConcurrent, 1)
				}
				if elapsed > 200_000 {
					atomic.AddInt64(&over200us, 1)
				}
				if elapsed > 500_000 {
					atomic.AddInt64(&over500us, 1)
				}
				if elapsed > 1_000_000 {
					atomic.AddInt64(&over1ms, 1)
				}
			}
		}(int64(w*1000 + 1))
	}
	wg.Wait()
	wallTime := time.Since(benchStart)
	totalReq := concurrency * pathsPerWorker

	fmt.Printf("Concurrency: %d, Requests/worker: %d, Total: %d\n", concurrency, pathsPerWorker, totalReq)
	fmt.Printf("Wall time: %.2f ms\n", float64(wallTime.Nanoseconds())/1e6)
	fmt.Printf("Throughput: %.0f req/s\n", float64(totalReq)/wallTime.Seconds())
	fmt.Printf("Success: %d/%d (%.1f%%)\n", successConcurrent, totalConcurrent,
		100.0*float64(successConcurrent)/float64(totalConcurrent))
	fmt.Printf("Avg compute: %.2f us\n",
		float64(totalConcurrentTimeNs)/float64(totalConcurrent)/1e3)
	fmt.Printf("  >200us: %d (%.1f%%), >500us: %d (%.1f%%), >1ms: %d (%.3f%%)\n",
		over200us, 100.0*float64(over200us)/float64(totalConcurrent),
		over500us, 100.0*float64(over500us)/float64(totalConcurrent),
		over1ms, 100.0*float64(over1ms)/float64(totalConcurrent))

	fmt.Println("\n======= Topology Refresh Benchmark =======")
	refreshStart := time.Now()
	snapMgr.Active()
	ephCfg := ephemeris.DefaultConstellationConfig()
	topoCfg := topology.DefaultTopologyConfig()
	eph := ephemeris.NewEphemerisModel(ephCfg)
	tb := topology.NewTopologyBuilder(topoCfg)
	epoch := uint64(time.Now().UnixMilli())
	posStart := time.Now()
	pos := eph.ComputeAllPositions(epoch)
	posTime := time.Since(posStart)
	buildStart := time.Now()
	newG := tb.Build(pos, posTime.Nanoseconds())
	buildTime := time.Since(buildStart)
	totalRefresh := time.Since(refreshStart)
	_ = newG

	fmt.Printf("Position compute: %.2f ms\n", float64(posTime.Nanoseconds())/1e6)
	fmt.Printf("Topology build:   %.2f ms (target: < 10000 ms)\n", float64(buildTime.Nanoseconds())/1e6)
	fmt.Printf("Total refresh:    %.2f ms\n", float64(totalRefresh.Nanoseconds())/1e6)
	fmt.Printf("New topology: %d links, avg degree %.2f\n", newG.TotalLinks(), newG.AvgDegree())
	_ = active
}
