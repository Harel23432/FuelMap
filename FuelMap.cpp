#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace std;

// This turns from engine specific into a control framework
struct SystemState {
	virtual ~SystemState() = default; // takes in SystemState and returns a control output
};

struct ControlOutput {
    double value; // control signal 
};

class Controller {
public:
	virtual ~Controller() = default;
	virtual ControlOutput compute(const SystemState& state) const = 0;
};

// --- Generic 2D Lookup Table ---

template<typename T>
class LookupTable2D {
private:
    const int* xAxis;
    const int* yAxis;
    int xSize;
    int ySize;
    const T* table;

public:
    LookupTable2D(const int* x, int xs, int* y, int ys, const T* t) : xAxis(x), yAxis(y), xSize(xs), ySize(ys), table(t) {}
};

// compile time constants 
constexpr int RPM_POINTS = 6;
constexpr int LOAD_POINTS = 5;

// simple data holder
struct EngineState {
    int rpm;
    int map; // in kPa
    double airMass; // grams per cycle
    double coolantTempC; // for cold start enrichment 
    double measuredAFR; // for closed loop correction 
};

// the injector
class Injector {
private:
    double flowRate; // grams per millisecond
public:
    explicit Injector(double flow) : flowRate(flow) {
        if (flowRate <= 0.0)
            throw invalid_argument("invalid injector flow");
    }

    double pulseWidth(double fuelMass) const {
        return fuelMass / flowRate;
    }
};

// the fuel map itself
class FuelMap {
private:
    const int* rpmAxis;
    const int* loadAxis;
    static constexpr int TABLE_SIZE = RPM_POINTS * LOAD_POINTS;
    double afrTable[TABLE_SIZE];

    int lowerIndex(const int* axis, int size, int value) const {
        if (value <= axis[0]) return 0;
        if (value >= axis[size - 1]) return size - 2;
        for (int i = 0; i < size - 1; ++i) {
            if (value >= axis[i] && value <= axis[i + 1])
                return i;
        }
        return 0;
    }

    double lerp(double a, double b, double t) const {
        return a + t * (b - a);
    }

    int index(int loadIndex, int rpmIndex) const {
        return loadIndex * RPM_POINTS + rpmIndex;
    }

    // --- Cold start enrichment ---
    double applyColdStartEnrichment(double afr, double coolantTempC) const {
        if (coolantTempC >= 90.0) return afr;
        double factor = 1.3 - (coolantTempC / 90.0) * 0.3; // linear enrichment 
        return afr * factor;
    }

    // --- closed loop O2 correction ---
    double applyClosedLoopCorrection(double afr, double measuredAFR) const {
        double error = afr - measuredAFR;
        double correctionFactor = 1.0 - 0.1 * error; // 0.1 = sensitivity
        return afr * correctionFactor;
    }

public:
    FuelMap(const int* rpm, const int* load, const double* table) : rpmAxis(rpm), loadAxis(load) {
        for (int i = 0; i < TABLE_SIZE; ++i) {
            afrTable[i] = table[i];
        }
    }

    double targetAFR(int rpm, int load) const {
        int r = lowerIndex(rpmAxis, RPM_POINTS, rpm);
        int l = lowerIndex(loadAxis, LOAD_POINTS, load);

        double rT = double(rpm - rpmAxis[r]) / double(rpmAxis[r + 1] - rpmAxis[r]);
        double lT = double(load - loadAxis[l]) / double(loadAxis[l + 1] - loadAxis[l]);

        double q11 = afrTable[index(l, r)];
        double q12 = afrTable[index(l, r + 1)];
        double q21 = afrTable[index(l + 1, r)];
        double q22 = afrTable[index(l + 1, r + 1)];

        double r1 = lerp(q11, q12, rT);
        double r2 = lerp(q21, q22, rT);

        return lerp(r1, r2, lT);
    }

    // --- combined function to get final AFR with cold start and closed loop ---
    // doesnt store the engine map. Uses only fuel map and injector for calculations 
    double getFinalAFR(int rpm, int load, double coolantTempC, double measuredAFR) const {
        double afr = targetAFR(rpm, load);
        afr = applyColdStartEnrichment(afr, coolantTempC); // adjusts the raw AFR for engine temp
        afr = applyClosedLoopCorrection(afr, measuredAFR); // further adjusts AFR based on real time O2 sensor feedback
        return afr;
    }
};

// --- FuelController using final AFR ---
class FuelController {
private:
    FuelMap fuelMap;
    Injector injector;

public:
    FuelController(FuelMap map, Injector inj) : fuelMap(map), injector(inj) {}

    double computePulseWidth(const EngineState& state) const {
        double afr = fuelMap.getFinalAFR(state.rpm, state.map, state.coolantTempC, state.measuredAFR);
        double fuelMass = state.airMass / afr;
        return injector.pulseWidth(fuelMass);
    }
};

// adapter that plugs engine logic into the framework
class EngineFuelController : public Controller { // inheritence
private:
    FuelController fuelController;

public:
    explicit EngineFuelController(const FuelController& fc) : fuelController(fc) {}

    ControlOutput compute(const SystemState& state) const override {
        const EngineState& engineState = dynamic_cast<const EngineState&>(state); // This lets you plug in any other system later, without touching the controller
        double pulseWidth = fuelController.computePulseWidth(engineState);

        return ControlOutput{ pulseWidth };
    }
};

//main function
int main() {
    static const int rpmAxis[RPM_POINTS] = { 1000, 2000, 3000, 4000, 5000, 6000 };
    static const int loadAxis[LOAD_POINTS] = { 20, 40, 60, 80, 100 };

    static const double afrTable[LOAD_POINTS * RPM_POINTS] = {
        //row 0
        14.7, 14.7, 14.7, 14.7, 14.7, 14.7,
        //row 1
        14.3, 14.1, 13.9, 13.7, 13.6, 13.6,
        //row 2
        13.6, 13.3, 13.0, 12.8, 12.8, 12.8,
        //row 3
        12.9, 12.6, 12.3, 12.0, 12.0, 12.0,
        //row 4
        12.2, 12.0, 11.8, 11.6, 11.5, 11.5
    };

    FuelMap map(rpmAxis, loadAxis, afrTable);
    Injector injector(0.02); //g/ms

    FuelController controller(map, injector);

    EngineState state{ 3500, 80, 0.45, 20.0, 14.0 };

    cout << "Injector Pulse Width: " << controller.computePulseWidth(state) << " ms" << endl;

    return 0;
}