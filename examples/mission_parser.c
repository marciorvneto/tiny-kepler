#define TINY_KEPLER_IMPLEMENTATION
#define TINY_LA_IMPLEMENTATION
#include <stddef.h>
#include <tinyla.h>
#include <tiny-kepler.h>

int main(){

	Arena a = arena_create(10 * 1024 * 1024); // 1MB
	const char *path = "./examples/missions/reduced-3-body.mission";
	MissionDescription mission = read_mission(&a, path);
	// print_mission(&mission);
	SimResults results = run_mission(&a, &mission);
	dump_cr3bp_result("./results.out", &results.as.cr3bp);

	arena_destroy(&a);
	return 0;
}
