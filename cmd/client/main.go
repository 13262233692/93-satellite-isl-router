package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"math/rand"
	"sync/atomic"
	"time"

	pb "satellite-isl-router/proto"

	grpc "google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
)

func main() {
	addr := flag.String("addr", "localhost:50051", "gRPC server address")
	nPaths := flag.Int("n", 100, "Number of paths to compute")
	satCount := flag.Int("sats", 10000, "Total satellites in constellation")
	concurrency := flag.Int("c", 16, "Concurrent requests")
	flag.Parse()

	conn, err := grpc.Dial(*addr, grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		log.Fatalf("fail to dial: %v", err)
	}
	defer conn.Close()

	client := pb.NewRouterServiceClient(conn)

	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	info, err := client.GetTopologyInfo(ctx, &pb.TopologyInfoRequest{RequestId: 1})
	if err != nil {
		log.Fatalf("GetTopologyInfo error: %v", err)
	}
	fmt.Printf("=== Server Topology Info ===\n")
	fmt.Printf("  Snapshot epoch: %d\n", info.GetSnapshotEpoch())
	fmt.Printf("  Satellites: %d\n", info.GetTotalSatellites())
	fmt.Printf("  Links: %d\n", info.GetTotalLinks())
	fmt.Printf("  Avg degree: %.2f\n", info.GetAvgDegree())
	fmt.Printf("  Build time: %.2f ms\n", float64(info.GetTopologyBuildTimeNs())/1e6)
	fmt.Println()

	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	fmt.Printf("=== Sequential Path Find (%d requests) ===\n", *nPaths)

	var totalTime int64 = 0
	var success int32 = 0
	var totalHops int32 = 0
	var totalDelayMs float64 = 0
	maxUs := int64(0)
	minUs := int64(1 << 62)

	for i := 0; i < *nPaths; i++ {
		src := uint64(rng.Intn(*satCount)) + 1
		dst := uint64(rng.Intn(*satCount)) + 1
		for src == dst {
			dst = uint64(rng.Intn(*satCount)) + 1
		}
		start := time.Now()
		reqCtx, reqCancel := context.WithTimeout(ctx, 5*time.Second)
		resp, err := client.FindPath(reqCtx, &pb.PathRequest{
			SourceSatelliteId:      src,
			DestinationSatelliteId: dst,
			RequestId:              uint64(i + 1),
			PreferMinHops:          false,
		})
		elapsedUs := time.Since(start).Microseconds()
		reqCancel()

		atomic.AddInt64(&totalTime, elapsedUs)
		if elapsedUs > maxUs {
			maxUs = elapsedUs
		}
		if elapsedUs < minUs {
			minUs = elapsedUs
		}

		if err != nil {
			if i < 3 {
				log.Printf("  Request %d (%d->%d) error: %v", i+1, src, dst, err)
			}
			continue
		}
		if !resp.GetPathFound() {
			if i < 3 {
				log.Printf("  Request %d (%d->%d): path not found (%s)", i+1, src, dst, resp.GetErrorMessage())
			}
			continue
		}
		atomic.AddInt32(&success, 1)
		atomic.AddInt32(&totalHops, int32(resp.GetTotalHops()))
		totalDelayMs += resp.GetTotalPropagationDelayMs()

		if i < 5 {
			fmt.Printf("  Request %d: %d->%d in %d hops, %.1f km, %.3f ms (computed in %d us)\n",
				i+1, src, dst, resp.GetTotalHops(),
				resp.GetTotalDistanceKm(), resp.GetTotalPropagationDelayMs(),
				elapsedUs)
			fmt.Printf("    Path: ")
			for idx, hop := range resp.GetPath() {
				if idx > 0 {
					fmt.Printf("->")
				}
				fmt.Printf("%d", hop.GetSatelliteId())
				if idx > 6 {
					fmt.Printf("...(+%d more)", len(resp.GetPath())-idx-1)
					break
				}
			}
			fmt.Println()
		}
	}

	nReal := *nPaths
	fmt.Printf("\nSummary:\n")
	fmt.Printf("  Success: %d/%d (%.1f%%)\n", success, nReal, 100.0*float64(success)/float64(nReal))
	if success > 0 {
		fmt.Printf("  Compute time: avg=%d us, max=%d us, min=%d us\n",
			atomic.LoadInt64(&totalTime)/int64(nReal), maxUs, minUs)
		fmt.Printf("  Avg hops: %.1f, Avg propagation delay: %.3f ms\n",
			float64(atomic.LoadInt32(&totalHops))/float64(success),
			totalDelayMs/float64(success))
	}

	fmt.Printf("\n=== Concurrent Path Find (%d workers x %d requests) ===\n", *concurrency, *nPaths)

	startAll := time.Now()
	var doneCount int64 = 0
	var concSuccess int64 = 0
	var concTotalUs int64 = 0
	var concOver1ms int64 = 0

	workerCh := make(chan struct{}, *concurrency)
	for w := 0; w < *concurrency; w++ {
		workerCh <- struct{}{}
	}

	benchStart := time.Now()
	go func() {
		for i := 0; i < *concurrency**nPaths; i++ {
			<-workerCh
			go func(reqIdx int) {
				defer func() { workerCh <- struct{}{} }()
				src := uint64(rand.Intn(*satCount)) + 1
				dst := uint64(rand.Intn(*satCount)) + 1
				for src == dst {
					dst = uint64(rand.Intn(*satCount)) + 1
				}
				start := time.Now()
				reqCtx, reqCancel := context.WithTimeout(ctx, 5*time.Second)
				resp, err := client.FindPath(reqCtx, &pb.PathRequest{
					SourceSatelliteId:      src,
					DestinationSatelliteId: dst,
					RequestId:              uint64(reqIdx + 1),
				})
				elapsedUs := time.Since(start).Microseconds()
				reqCancel()

				atomic.AddInt64(&concTotalUs, elapsedUs)
				atomic.AddInt64(&doneCount, 1)
				if elapsedUs > 1_000_000/1000 {
					atomic.AddInt64(&concOver1ms, 1)
				}
				if err == nil && resp != nil && resp.GetPathFound() {
					atomic.AddInt64(&concSuccess, 1)
				}
			}(i)
		}
	}()

	total := int64(*concurrency * *nPaths)
	for atomic.LoadInt64(&doneCount) < total {
		time.Sleep(200 * time.Millisecond)
		if time.Since(benchStart) > 60*time.Second {
			log.Printf("Timeout waiting for benchmark")
			break
		}
	}

	wallTime := time.Since(startAll)
	done := atomic.LoadInt64(&doneCount)
	fmt.Printf("  Wall time: %.2f s\n", wallTime.Seconds())
	fmt.Printf("  Completed: %d/%d\n", done, total)
	fmt.Printf("  Throughput: %.0f req/s\n", float64(done)/wallTime.Seconds())
	fmt.Printf("  Success: %d (%.1f%%)\n", concSuccess, 100.0*float64(concSuccess)/float64(done))
	if done > 0 {
		fmt.Printf("  Avg compute: %d us, >1ms: %d (%.1f%%)\n",
			atomic.LoadInt64(&concTotalUs)/done,
			concOver1ms, 100.0*float64(concOver1ms)/float64(done))
	}

	fmt.Println("\n=== Done ===")
}
