#define TINY_KEPLER_IMPLEMENTATION
#define TINY_LA_IMPLEMENTATION
#include <stddef.h>
#include <tinyla.h>
#include <tiny-kepler.h>

int main(int argc, char **argv){
	Arena a = arena_create(10 * 1024 * 1024); // 1MB

	const char *path = argc > 1 ? argv[1] : "./examples/missions/restricted-3-body.mission";
	const char *out_path = argc > 2 ? argv[2] : "./results.out";
	MissionDescription mission = read_mission(&a, path);
	// print_mission(&mission);
	SimResults results = run_mission(&a, &mission);
	dump_cr3bp_result(out_path, &results.as.cr3bp);

	arena_destroy(&a);
	return 0;
}
