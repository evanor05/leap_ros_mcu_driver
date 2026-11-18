#include "sample_clock.h"
#include <cassert>
int main() {
    SampleClock clock; SamplePublishGate gate;
    SampleMetadata sample; sample.valid=true; sample.sequence=1; sample.sample_time_us=1100000;
    clock.Reset(1000000);
    assert(clock.Stamp(sample,1200000)==0); // No epoch mapping.
    clock.Synchronize(1200000,1700000000200LL);
    const auto stamp=clock.Stamp(sample,1200000);
    assert(stamp==1700000000100000000LL);
    assert(gate.New(stamp));
    assert(gate.New(stamp)); // Failed send may retry with the same source time.
    gate.Published(stamp);assert(!gate.New(stamp));
    assert(clock.Stamp(sample,1400001)==0); // Sampling frozen while transport lives.
    sample.valid=false;assert(clock.Stamp(sample,1200000)==0);
    sample.valid=true;assert(clock.Stamp(sample,1000000)==0); // Future sample.
    clock.Reset(1300000);gate.Reset();clock.Synchronize(1400000,1700000000400LL);
    assert(clock.Stamp(sample,1400000)==0); // Reconnect rejects previous-session cache.
    sample.sample_time_us=1350000;++sample.sequence;
    assert(gate.New(clock.Stamp(sample,1400000)));
}
