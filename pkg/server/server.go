package server

import (
	"context"
	"fmt"
	"time"

	pb "satellite-isl-router/proto"
	"satellite-isl-router/pkg/snapshot"
)

type RouterGRPCServer struct {
	pb.UnimplementedRouterServiceServer
	snapMgr *snapshot.Manager
	useAStar bool
}

func NewRouterGRPCServer(snapMgr *snapshot.Manager, useAStar bool) *RouterGRPCServer {
	return &RouterGRPCServer{
		snapMgr: snapMgr,
		useAStar: useAStar,
	}
}

func (s *RouterGRPCServer) FindPath(ctx context.Context, req *pb.PathRequest) (*pb.PathResponse, error) {
	start := time.Now()
	resp := &pb.PathResponse{
		RequestId: req.GetRequestId(),
	}

	srcID := req.GetSourceSatelliteId()
	dstID := req.GetDestinationSatelliteId()
	preferMinHops := req.GetPreferMinHops()

	activeSnap := s.snapMgr.Active()
	resp.SnapshotEpoch = activeSnap.EpochUnixMs

	if srcID == 0 || dstID == 0 {
		resp.ErrorMessage = "invalid satellite id"
		resp.ComputeTimeNs = time.Since(start).Nanoseconds()
		return resp, nil
	}

	if activeSnap.Graph.SatCount == 0 {
		resp.ErrorMessage = "topology not ready"
		resp.ComputeTimeNs = time.Since(start).Nanoseconds()
		return resp, nil
	}

	_, srcOk := activeSnap.Graph.SatIDToIndex[srcID]
	_, dstOk := activeSnap.Graph.SatIDToIndex[dstID]
	if !srcOk || !dstOk {
		resp.ErrorMessage = fmt.Sprintf("satellite not found: src=%v src_ok=%v dst=%v dst_ok=%v", srcID, srcOk, dstID, dstOk)
		resp.ComputeTimeNs = time.Since(start).Nanoseconds()
		return resp, nil
	}

	r := activeSnap.GetRouter()
	result := r.FindPath(srcID, dstID, preferMinHops, s.useAStar)

	if !result.Found {
		resp.PathFound = false
		resp.ErrorMessage = "path not found"
		resp.ComputeTimeNs = time.Since(start).Nanoseconds()
		return resp, nil
	}

	hops := make([]*pb.PathHop, len(result.PathSatIDs))
	for i := 0; i < len(result.PathSatIDs); i++ {
		hops[i] = &pb.PathHop{
			SatelliteId:        result.PathSatIDs[i],
			HopIndex:           int32(i),
			LinkDistanceKm:     result.LinkDistancesKm[i],
			PropagationDelayMs: result.LinkDelaysMs[i],
		}
	}
	resp.PathFound = true
	resp.Path = hops
	resp.TotalHops = int32(result.TotalHops)
	resp.TotalDistanceKm = result.TotalDistanceKm
	resp.TotalPropagationDelayMs = result.TotalPropagationDelayMs
	resp.ComputeTimeNs = time.Since(start).Nanoseconds()

	return resp, nil
}

func (s *RouterGRPCServer) GetTopologyInfo(ctx context.Context, req *pb.TopologyInfoRequest) (*pb.TopologyInfoResponse, error) {
	resp := &pb.TopologyInfoResponse{
		RequestId: req.GetRequestId(),
	}
	activeSnap := s.snapMgr.Active()
	g := activeSnap.Graph
	resp.SnapshotEpoch = activeSnap.EpochUnixMs
	resp.TotalSatellites = int32(g.SatCount)
	resp.TotalLinks = int32(g.TotalLinks())
	resp.AvgDegree = g.AvgDegree()
	resp.TopologyBuildTimeNs = g.BuildTimeNs
	return resp, nil
}

func (s *RouterGRPCServer) GetSatellitePosition(ctx context.Context, req *pb.SatellitePositionRequest) (*pb.SatellitePositionResponse, error) {
	resp := &pb.SatellitePositionResponse{
		RequestId:   req.GetRequestId(),
		SatelliteId: req.GetSatelliteId(),
	}
	activeSnap := s.snapMgr.Active()
	resp.SnapshotEpoch = activeSnap.EpochUnixMs

	id := req.GetSatelliteId()
	pos, ok := activeSnap.Positions.GetPosition(id)
	if !ok {
		return resp, fmt.Errorf("satellite %v not found", id)
	}
	resp.XKm = pos.XKm
	resp.YKm = pos.YKm
	resp.ZKm = pos.ZKm
	resp.AltitudeKm = pos.AltitudeKm
	return resp, nil
}

type UnimplementedRouterServiceServer struct{}

func (*UnimplementedRouterServiceServer) FindPath(ctx context.Context, req *pb.PathRequest) (*pb.PathResponse, error) {
	return nil, fmt.Errorf("not implemented")
}
func (*UnimplementedRouterServiceServer) GetTopologyInfo(ctx context.Context, req *pb.TopologyInfoRequest) (*pb.TopologyInfoResponse, error) {
	return nil, fmt.Errorf("not implemented")
}
func (*UnimplementedRouterServiceServer) GetSatellitePosition(ctx context.Context, req *pb.SatellitePositionRequest) (*pb.SatellitePositionResponse, error) {
	return nil, fmt.Errorf("not implemented")
}
