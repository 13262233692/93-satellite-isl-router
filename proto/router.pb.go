package proto

type PathRequest struct {
	SourceSatelliteId      uint64
	DestinationSatelliteId uint64
	RequestId              uint64
	PreferMinHops          bool
}

func (x *PathRequest) GetSourceSatelliteId() uint64      { if x != nil { return x.SourceSatelliteId }; return 0 }
func (x *PathRequest) GetDestinationSatelliteId() uint64 { if x != nil { return x.DestinationSatelliteId }; return 0 }
func (x *PathRequest) GetRequestId() uint64              { if x != nil { return x.RequestId }; return 0 }
func (x *PathRequest) GetPreferMinHops() bool            { if x != nil { return x.PreferMinHops }; return false }

type PathHop struct {
	SatelliteId        uint64
	HopIndex           int32
	LinkDistanceKm     float64
	PropagationDelayMs float64
}

func (x *PathHop) GetSatelliteId() uint64          { if x != nil { return x.SatelliteId }; return 0 }
func (x *PathHop) GetHopIndex() int32              { if x != nil { return x.HopIndex }; return 0 }
func (x *PathHop) GetLinkDistanceKm() float64      { if x != nil { return x.LinkDistanceKm }; return 0 }
func (x *PathHop) GetPropagationDelayMs() float64  { if x != nil { return x.PropagationDelayMs }; return 0 }

type PathResponse struct {
	RequestId               uint64
	PathFound               bool
	Path                    []*PathHop
	TotalHops               int32
	TotalDistanceKm         float64
	TotalPropagationDelayMs float64
	SnapshotEpoch           uint64
	ComputeTimeNs           int64
	ErrorMessage            string
}

func (x *PathResponse) GetRequestId() uint64                 { if x != nil { return x.RequestId }; return 0 }
func (x *PathResponse) GetPathFound() bool                   { if x != nil { return x.PathFound }; return false }
func (x *PathResponse) GetPath() []*PathHop                  { if x != nil { return x.Path }; return nil }
func (x *PathResponse) GetTotalHops() int32                  { if x != nil { return x.TotalHops }; return 0 }
func (x *PathResponse) GetTotalDistanceKm() float64          { if x != nil { return x.TotalDistanceKm }; return 0 }
func (x *PathResponse) GetTotalPropagationDelayMs() float64  { if x != nil { return x.TotalPropagationDelayMs }; return 0 }
func (x *PathResponse) GetSnapshotEpoch() uint64             { if x != nil { return x.SnapshotEpoch }; return 0 }
func (x *PathResponse) GetComputeTimeNs() int64              { if x != nil { return x.ComputeTimeNs }; return 0 }
func (x *PathResponse) GetErrorMessage() string              { if x != nil { return x.ErrorMessage }; return "" }

type TopologyInfoRequest struct { RequestId uint64 }
func (x *TopologyInfoRequest) GetRequestId() uint64 { if x != nil { return x.RequestId }; return 0 }

type TopologyInfoResponse struct {
	RequestId           uint64
	SnapshotEpoch       uint64
	TotalSatellites     int32
	TotalLinks          int32
	AvgDegree           float64
	TopologyBuildTimeNs int64
}
func (x *TopologyInfoResponse) GetRequestId() uint64           { if x != nil { return x.RequestId }; return 0 }
func (x *TopologyInfoResponse) GetSnapshotEpoch() uint64       { if x != nil { return x.SnapshotEpoch }; return 0 }
func (x *TopologyInfoResponse) GetTotalSatellites() int32      { if x != nil { return x.TotalSatellites }; return 0 }
func (x *TopologyInfoResponse) GetTotalLinks() int32           { if x != nil { return x.TotalLinks }; return 0 }
func (x *TopologyInfoResponse) GetAvgDegree() float64          { if x != nil { return x.AvgDegree }; return 0 }
func (x *TopologyInfoResponse) GetTopologyBuildTimeNs() int64  { if x != nil { return x.TopologyBuildTimeNs }; return 0 }

type SatellitePositionRequest struct {
	SatelliteId uint64
	RequestId   uint64
}
func (x *SatellitePositionRequest) GetSatelliteId() uint64 { if x != nil { return x.SatelliteId }; return 0 }
func (x *SatellitePositionRequest) GetRequestId() uint64   { if x != nil { return x.RequestId }; return 0 }

type SatellitePositionResponse struct {
	RequestId     uint64
	SatelliteId   uint64
	XKm           float64
	YKm           float64
	ZKm           float64
	AltitudeKm    float64
	SnapshotEpoch uint64
}
func (x *SatellitePositionResponse) GetRequestId() uint64     { if x != nil { return x.RequestId }; return 0 }
func (x *SatellitePositionResponse) GetSatelliteId() uint64   { if x != nil { return x.SatelliteId }; return 0 }
func (x *SatellitePositionResponse) GetXKm() float64          { if x != nil { return x.XKm }; return 0 }
func (x *SatellitePositionResponse) GetYKm() float64          { if x != nil { return x.YKm }; return 0 }
func (x *SatellitePositionResponse) GetZKm() float64          { if x != nil { return x.ZKm }; return 0 }
func (x *SatellitePositionResponse) GetAltitudeKm() float64   { if x != nil { return x.AltitudeKm }; return 0 }
func (x *SatellitePositionResponse) GetSnapshotEpoch() uint64 { if x != nil { return x.SnapshotEpoch }; return 0 }
