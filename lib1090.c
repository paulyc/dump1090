#include "lib1090.h"

void lib1090Init(float userLat, float userLon, float userAltMeters) {
    modesInitConfig();
    install_signal_handlers(false);

    Modes.sdr_type = SDR_NONE;
    Modes.fUserLat = userLat;
    Modes.fUserLon = userLon;
    Modes.fUserAltM = userAltMeters;
    Modes.json_dir = "/var/cache/piaware";

    modesInit();
    modesInitNet();
    modesInitStats();
}

void lib1090Run() {
}
