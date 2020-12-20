

extern float InitialOpening;        // Initial opening of barn doors at home position
                                    // (distance between the two pivot points in cm)
extern float MaximumOpening;        // Maximum distance to allow barn doors to open (30 deg == 2 hours)
extern long PlanAheadTrueTimeMs;    // Real time in ms between plans
extern long PlanAheadSysTimeMs;     // System time in ms between plans


void readSettings();
void saveSettings();

void init_vars();