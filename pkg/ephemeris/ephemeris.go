package ephemeris

import (
	"math"
	"sync"
)

const (
	EarthRadiusKm   = 6371.0
	SpeedOfLightKms = 299792.458
	MuEarthKm3s2    = 398600.4418
	Deg2Rad         = math.Pi / 180.0
	Rad2Deg         = 180.0 / math.Pi
)

type OrbitParams struct {
	SemimajorAxisKm float64
	InclinationRad  float64
	Eccentricity    float64
	RAANRad         float64
	ArgOfPerigeeRad float64
	MeanAnomalyRad  float64
}

type Satellite struct {
	ID             uint64
	OrbitPlane     int
	OrbitParams    OrbitParams
	MeanMotionRadS float64
	PeriodSeconds  float64
}

type ConstellationConfig struct {
	TotalSatellites int
	AltitudeKm      float64
	InclinationDeg  float64
	NumOrbitPlanes  int
	SatsPerPlane    int
	RAANDeltaDeg    float64
	PhaseShiftDeg   float64
}

func DefaultConstellationConfig() ConstellationConfig {
	return ConstellationConfig{
		TotalSatellites: 10000,
		AltitudeKm:      550.0,
		InclinationDeg:  53.0,
		NumOrbitPlanes:  100,
		SatsPerPlane:    100,
		RAANDeltaDeg:    3.6,
		PhaseShiftDeg:   1.0,
	}
}

type EphemerisModel struct {
	mu            sync.RWMutex
	config        ConstellationConfig
	satellites    []*Satellite
	satIndex      map[uint64]int
	semimajorAxis float64
}

func NewEphemerisModel(config ConstellationConfig) *EphemerisModel {
	if config.TotalSatellites <= 0 {
		config = DefaultConstellationConfig()
	}
	if config.NumOrbitPlanes*config.SatsPerPlane < config.TotalSatellites {
		config.NumOrbitPlanes = int(math.Sqrt(float64(config.TotalSatellites)))
		config.SatsPerPlane = config.TotalSatellites / config.NumOrbitPlanes
		if config.TotalSatellites%config.NumOrbitPlanes != 0 {
			config.SatsPerPlane++
		}
		config.RAANDeltaDeg = 180.0 / float64(config.NumOrbitPlanes)
	}

	sma := EarthRadiusKm + config.AltitudeKm

	em := &EphemerisModel{
		config:        config,
		satellites:    make([]*Satellite, 0, config.TotalSatellites),
		satIndex:      make(map[uint64]int, config.TotalSatellites),
		semimajorAxis: sma,
	}
	em.buildConstellation()
	return em
}

func (em *EphemerisModel) buildConstellation() {
	sma := em.semimajorAxis
	meanMotion := math.Sqrt(MuEarthKm3s2 / (sma * sma * sma))
	period := 2 * math.Pi / meanMotion

	satID := uint64(1)
	satsBuilt := 0

	for planeIdx := 0; planeIdx < em.config.NumOrbitPlanes && satsBuilt < em.config.TotalSatellites; planeIdx++ {
		raan := float64(planeIdx) * em.config.RAANDeltaDeg * Deg2Rad
		phaseOffset := float64(planeIdx%2) * em.config.PhaseShiftDeg * Deg2Rad

		satsInThisPlane := em.config.SatsPerPlane
		if satsBuilt+satsInThisPlane > em.config.TotalSatellites {
			satsInThisPlane = em.config.TotalSatellites - satsBuilt
		}

		for satInPlane := 0; satInPlane < satsInThisPlane; satInPlane++ {
			meanAnomaly := (2.0*math.Pi*float64(satInPlane)/float64(satsInThisPlane)) + phaseOffset
			for meanAnomaly >= 2*math.Pi {
				meanAnomaly -= 2 * math.Pi
			}

			sat := &Satellite{
				ID:         satID,
				OrbitPlane: planeIdx,
				OrbitParams: OrbitParams{
					SemimajorAxisKm: sma,
					InclinationRad:  em.config.InclinationDeg * Deg2Rad,
					Eccentricity:    0.0001,
					RAANRad:         raan,
					ArgOfPerigeeRad: 0.0,
					MeanAnomalyRad:  meanAnomaly,
				},
				MeanMotionRadS: meanMotion,
				PeriodSeconds:  period,
			}
			em.satellites = append(em.satellites, sat)
			em.satIndex[satID] = len(em.satellites) - 1
			satID++
			satsBuilt++
		}
	}
}

func (em *EphemerisModel) GetSatelliteCount() int {
	return len(em.satellites)
}

func (em *EphemerisModel) GetSatelliteIDs() []uint64 {
	em.mu.RLock()
	defer em.mu.RUnlock()
	ids := make([]uint64, len(em.satellites))
	for i, sat := range em.satellites {
		ids[i] = sat.ID
	}
	return ids
}

func (em *EphemerisModel) GetSatelliteByID(id uint64) *Satellite {
	em.mu.RLock()
	defer em.mu.RUnlock()
	idx, ok := em.satIndex[id]
	if !ok {
		return nil
	}
	return em.satellites[idx]
}

type ECEFPosition struct {
	XKm        float64
	YKm        float64
	ZKm        float64
	AltitudeKm float64
}

type PositionSnapshot struct {
	EpochUnixMs uint64
	Positions   []ECEFPosition
	IDToIndex   map[uint64]int
}

func (ps *PositionSnapshot) GetPosition(id uint64) (ECEFPosition, bool) {
	idx, ok := ps.IDToIndex[id]
	if !ok {
		return ECEFPosition{}, false
	}
	return ps.Positions[idx], true
}

func solveKepler(M, e float64) float64 {
	E := M
	if e < 1e-8 {
		return E
	}
	for i := 0; i < 15; i++ {
		dE := (E - e*math.Sin(E) - M) / (1 - e*math.Cos(E))
		E -= dE
		if math.Abs(dE) < 1e-12 {
			break
		}
	}
	return E
}

func (sat *Satellite) propagate(epochSeconds float64) ECEFPosition {
	op := sat.OrbitParams
	M := op.MeanAnomalyRad + sat.MeanMotionRadS*epochSeconds
	for M < 0 {
		M += 2 * math.Pi
	}
	for M >= 2*math.Pi {
		M -= 2 * math.Pi
	}

	E := solveKepler(M, op.Eccentricity)

	cosE := math.Cos(E)
	sinE := math.Sin(E)
	e := op.Eccentricity

	xOrb := op.SemimajorAxisKm * (cosE - e)
	yOrb := op.SemimajorAxisKm * math.Sqrt(1-e*e) * sinE

	cosRAAN := math.Cos(op.RAANRad)
	sinRAAN := math.Sin(op.RAANRad)
	cosI := math.Cos(op.InclinationRad)
	sinI := math.Sin(op.InclinationRad)
	cosW := math.Cos(op.ArgOfPerigeeRad)
	sinW := math.Sin(op.ArgOfPerigeeRad)

	x := (cosRAAN*cosW - sinRAAN*sinW*cosI) * xOrb + (-cosRAAN*sinW - sinRAAN*cosW*cosI) * yOrb
	y := (sinRAAN*cosW + cosRAAN*sinW*cosI) * xOrb + (-sinRAAN*sinW + cosRAAN*cosW*cosI) * yOrb
	z := (sinW * sinI) * xOrb + (cosW * sinI) * yOrb

	r := math.Sqrt(x*x + y*y + z*z)
	alt := r - EarthRadiusKm

	return ECEFPosition{
		XKm:        x,
		YKm:        y,
		ZKm:        z,
		AltitudeKm: alt,
	}
}

func (em *EphemerisModel) ComputeAllPositions(epochUnixMs uint64) *PositionSnapshot {
	epochSeconds := float64(epochUnixMs) / 1000.0

	em.mu.RLock()
	sats := em.satellites
	em.mu.RUnlock()

	snapshot := &PositionSnapshot{
		EpochUnixMs: epochUnixMs,
		Positions:   make([]ECEFPosition, len(sats)),
		IDToIndex:   make(map[uint64]int, len(sats)),
	}

	for i, sat := range sats {
		snapshot.Positions[i] = sat.propagate(epochSeconds)
		snapshot.IDToIndex[sat.ID] = i
	}

	return snapshot
}

func DistanceKm(p1, p2 ECEFPosition) float64 {
	dx := p1.XKm - p2.XKm
	dy := p1.YKm - p2.YKm
	dz := p1.ZKm - p2.ZKm
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func PropagationDelayMs(distanceKm float64) float64 {
	return (distanceKm / SpeedOfLightKms) * 1000.0
}
