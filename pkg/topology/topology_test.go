package topology

import (
	"math"
	"testing"
	"time"

	"satellite-isl-router/pkg/ephemeris"
)

func TestIsLineBlockedByEarth(t *testing.T) {
	R := ephemeris.EarthRadiusKm

	p1 := ephemeris.ECEFPosition{XKm: R + 550, YKm: 0, ZKm: 0}
	p2 := ephemeris.ECEFPosition{XKm: -(R + 550), YKm: 0, ZKm: 0}
	if !IsLineBlockedByEarth(p1, p2, 0) {
		t.Error("Opposite side of Earth should be blocked")
	}

	p3 := ephemeris.ECEFPosition{XKm: R + 550, YKm: 100, ZKm: 0}
	p4 := ephemeris.ECEFPosition{XKm: R + 550, YKm: 2000, ZKm: 0}
	if IsLineBlockedByEarth(p3, p4, 0) {
		t.Error("Nearby sats same side should not be blocked")
	}

	latDeg := 45.0 * math.Pi / 180.0
	p5 := ephemeris.ECEFPosition{
		XKm: (R + 550) * math.Cos(latDeg),
		YKm: 0,
		ZKm: (R + 550) * math.Sin(latDeg),
	}
	latDeg2 := -45.0 * math.Pi / 180.0
	p6 := ephemeris.ECEFPosition{
		XKm: (R + 550) * math.Cos(latDeg2),
		YKm: 0,
		ZKm: (R + 550) * math.Sin(latDeg2),
	}
	if !IsLineBlockedByEarth(p5, p6, 0) {
		t.Log("90 degree apart satellites are blocked by Earth (polar orbit through center)")
	}
}

func TestTopologyBuild1000(t *testing.T) {
	ephCfg := ephemeris.DefaultConstellationConfig()
	ephCfg.TotalSatellites = 1000
	ephCfg.NumOrbitPlanes = 25
	ephCfg.SatsPerPlane = 40
	ephCfg.RAANDeltaDeg = 180.0 / 25.0
	eph := ephemeris.NewEphemerisModel(ephCfg)
	if eph.GetSatelliteCount() != 1000 {
		t.Errorf("Expected 1000 sats, got %d", eph.GetSatelliteCount())
	}
	epoch := uint64(time.Now().UnixMilli())
	start := time.Now()
	pos := eph.ComputeAllPositions(epoch)
	t.Logf("Position compute: %d ms", time.Since(start).Milliseconds())

	tb := NewTopologyBuilder(DefaultTopologyConfig())
	start = time.Now()
	g := tb.Build(pos, time.Since(start).Nanoseconds())
	buildMs := time.Since(start).Milliseconds()

	t.Logf("Topology: %d sats, %d directed links, avg degree %.2f in %d ms",
		g.SatCount, g.TotalLinks(), g.AvgDegree(), buildMs)

	if g.SatCount != 1000 {
		t.Errorf("Expected 1000 sats in graph, got %d", g.SatCount)
	}
	minLinks := 1000 * 4
	if g.TotalLinks() < minLinks {
		t.Errorf("Expected at least %d links, got %d", minLinks, g.TotalLinks())
	}

	_, ok := g.GetSatIndex(g.SatIndexToID[0])
	if !ok {
		t.Error("Sat ID lookup failed")
	}
}

func TestDistance(t *testing.T) {
	p1 := ephemeris.ECEFPosition{XKm: 0, YKm: 0, ZKm: 0}
	p2 := ephemeris.ECEFPosition{XKm: 300, YKm: 400, ZKm: 0}
	d := ephemeris.DistanceKm(p1, p2)
	expected := 500.0
	if math.Abs(d-expected) > 0.01 {
		t.Errorf("Distance expected %.3f, got %.3f", expected, d)
	}
	delay := ephemeris.PropagationDelayMs(d)
	expectedDelay := (500.0 / 299792.458) * 1000
	if math.Abs(delay-expectedDelay) > 0.0001 {
		t.Errorf("Delay expected %.6f ms, got %.6f ms", expectedDelay, delay)
	}
	t.Logf("500km delay: %.6f ms", delay)
}

func TestRelativeRadialVelocity(t *testing.T) {
	p1 := ephemeris.ECEFPosition{XKm: 0, YKm: 0, ZKm: 0}
	p2 := ephemeris.ECEFPosition{XKm: 100, YKm: 0, ZKm: 0}

	v1 := ephemeris.ECEFVelocity{VxKmS: 1, VyKmS: 0, VzKmS: 0}
	v2 := ephemeris.ECEFVelocity{VxKmS: 6, VyKmS: 0, VzKmS: 0}
	rv := ephemeris.RelativeRadialVelocityKms(p1, p2, v1, v2)
	if math.Abs(rv-5.0) > 1e-6 {
		t.Errorf("Expected 5.0 km/s approaching, got %.6f", rv)
	}
	t.Logf("Co-linear approach: %.2f km/s", rv)

	v2b := ephemeris.ECEFVelocity{VxKmS: -4, VyKmS: 0, VzKmS: 0}
	rv2 := ephemeris.RelativeRadialVelocityKms(p1, p2, v1, v2b)
	if math.Abs(rv2+5.0) > 1e-6 {
		t.Errorf("Expected -5.0 km/s separating, got %.6f", rv2)
	}
	t.Logf("Co-linear separate: %.2f km/s", rv2)

	p3 := ephemeris.ECEFPosition{XKm: 0, YKm: 0, ZKm: 0}
	p4 := ephemeris.ECEFPosition{XKm: 100, YKm: 0, ZKm: 0}
	v3 := ephemeris.ECEFVelocity{VxKmS: 0, VyKmS: 10, VzKmS: 0}
	v4 := ephemeris.ECEFVelocity{VxKmS: 0, VyKmS: -5, VzKmS: 0}
	rvPerp := ephemeris.RelativeRadialVelocityKms(p3, p4, v3, v4)
	if math.Abs(rvPerp) > 1e-6 {
		t.Errorf("Perpendicular motion should have 0 radial velocity, got %.6f", rvPerp)
	}
	t.Logf("Perpendicular (0 radial): %.2f km/s", rvPerp)
}

func TestRadialVelocityCullsLinks(t *testing.T) {
	ephCfg := ephemeris.DefaultConstellationConfig()
	ephCfg.TotalSatellites = 1000
	ephCfg.NumOrbitPlanes = 25
	ephCfg.SatsPerPlane = 40
	ephCfg.RAANDeltaDeg = 180.0 / 25.0
	eph := ephemeris.NewEphemerisModel(ephCfg)
	epoch := uint64(time.Now().UnixMilli())
	pos := eph.ComputeAllPositions(epoch)

	tbUnlimited := NewTopologyBuilder(TopologyConfig{
		MaxLaserLinkKm:       5000,
		OcclusionMarginKm:    50,
		GridCellSizeKm:       800,
		MaxLinksPerSat:       20,
		MaxRadialVelocityKms: 100,
	})
	gAll := tbUnlimited.Build(pos, 0)

	tbCulled := NewTopologyBuilder(TopologyConfig{
		MaxLaserLinkKm:       5000,
		OcclusionMarginKm:    50,
		GridCellSizeKm:       800,
		MaxLinksPerSat:       20,
		MaxRadialVelocityKms: 1.0,
	})
	gCulled := tbCulled.Build(pos, 0)

	t.Logf("Unlimited (100 km/s threshold): %d links, avg deg %.2f",
		gAll.TotalLinks(), gAll.AvgDegree())
	t.Logf("Strict (1.0 km/s threshold):    %d links, avg deg %.2f",
		gCulled.TotalLinks(), gCulled.AvgDegree())
	t.Logf("Culled ratio: %.1f%% links removed by Doppler constraint",
		100.0*float64(gAll.TotalLinks()-gCulled.TotalLinks())/float64(gAll.TotalLinks()))

	if gCulled.TotalLinks() >= gAll.TotalLinks() {
		t.Error("Strict Doppler threshold should reduce link count")
	}
	if gCulled.AvgDegree() < 2.0 {
		t.Errorf("Avg degree too low after Doppler cull: %.2f", gCulled.AvgDegree())
	}
}
