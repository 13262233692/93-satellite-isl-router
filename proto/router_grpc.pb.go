package proto

import (
	context "context"
	"fmt"
	grpc "google.golang.org/grpc"
)

const (
	RouterService_FindPath_FullMethodName            = "/satellite_isl.RouterService/FindPath"
	RouterService_GetTopologyInfo_FullMethodName     = "/satellite_isl.RouterService/GetTopologyInfo"
	RouterService_GetSatellitePosition_FullMethodName = "/satellite_isl.RouterService/GetSatellitePosition"
)

type RouterServiceServer interface {
	FindPath(context.Context, *PathRequest) (*PathResponse, error)
	GetTopologyInfo(context.Context, *TopologyInfoRequest) (*TopologyInfoResponse, error)
	GetSatellitePosition(context.Context, *SatellitePositionRequest) (*SatellitePositionResponse, error)
}

func RegisterRouterServiceServer(s grpc.ServiceRegistrar, srv RouterServiceServer) {
	s.RegisterService(&RouterService_ServiceDesc, srv)
}

func _RouterService_FindPath_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(PathRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RouterServiceServer).FindPath(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: RouterService_FindPath_FullMethodName,
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RouterServiceServer).FindPath(ctx, req.(*PathRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RouterService_GetTopologyInfo_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(TopologyInfoRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RouterServiceServer).GetTopologyInfo(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: RouterService_GetTopologyInfo_FullMethodName,
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RouterServiceServer).GetTopologyInfo(ctx, req.(*TopologyInfoRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _RouterService_GetSatellitePosition_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(SatellitePositionRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(RouterServiceServer).GetSatellitePosition(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: RouterService_GetSatellitePosition_FullMethodName,
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(RouterServiceServer).GetSatellitePosition(ctx, req.(*SatellitePositionRequest))
	}
	return interceptor(ctx, in, info, handler)
}

var RouterService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "satellite_isl.RouterService",
	HandlerType: (*RouterServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "FindPath",
			Handler:    _RouterService_FindPath_Handler,
		},
		{
			MethodName: "GetTopologyInfo",
			Handler:    _RouterService_GetTopologyInfo_Handler,
		},
		{
			MethodName: "GetSatellitePosition",
			Handler:    _RouterService_GetSatellitePosition_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "proto/router.proto",
}

type RouterServiceClient interface {
	FindPath(ctx context.Context, in *PathRequest, opts ...grpc.CallOption) (*PathResponse, error)
	GetTopologyInfo(ctx context.Context, in *TopologyInfoRequest, opts ...grpc.CallOption) (*TopologyInfoResponse, error)
	GetSatellitePosition(ctx context.Context, in *SatellitePositionRequest, opts ...grpc.CallOption) (*SatellitePositionResponse, error)
}

type routerServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewRouterServiceClient(cc grpc.ClientConnInterface) RouterServiceClient {
	return &routerServiceClient{cc}
}

func (c *routerServiceClient) FindPath(ctx context.Context, in *PathRequest, opts ...grpc.CallOption) (*PathResponse, error) {
	out := new(PathResponse)
	err := c.cc.Invoke(ctx, RouterService_FindPath_FullMethodName, in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *routerServiceClient) GetTopologyInfo(ctx context.Context, in *TopologyInfoRequest, opts ...grpc.CallOption) (*TopologyInfoResponse, error) {
	out := new(TopologyInfoResponse)
	err := c.cc.Invoke(ctx, RouterService_GetTopologyInfo_FullMethodName, in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *routerServiceClient) GetSatellitePosition(ctx context.Context, in *SatellitePositionRequest, opts ...grpc.CallOption) (*SatellitePositionResponse, error) {
	out := new(SatellitePositionResponse)
	err := c.cc.Invoke(ctx, RouterService_GetSatellitePosition_FullMethodName, in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

type UnimplementedRouterServiceServer struct{}

func (*UnimplementedRouterServiceServer) FindPath(ctx context.Context, req *PathRequest) (*PathResponse, error) {
	return nil, fmt.Errorf("method FindPath not implemented")
}
func (*UnimplementedRouterServiceServer) GetTopologyInfo(ctx context.Context, req *TopologyInfoRequest) (*TopologyInfoResponse, error) {
	return nil, fmt.Errorf("method GetTopologyInfo not implemented")
}
func (*UnimplementedRouterServiceServer) GetSatellitePosition(ctx context.Context, req *SatellitePositionRequest) (*SatellitePositionResponse, error) {
	return nil, fmt.Errorf("method GetSatellitePosition not implemented")
}
