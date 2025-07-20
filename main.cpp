#include "conway.hpp"

static std::unordered_map<std::string, std::string> InputMap{
	{"Block", R"(
        1 1
        2 1
        1 2
        2 2
    )"},
	{"SimpleBlinker", R"(
		1 1
		2 1
		3 1
	)"},
	{"BoundaryBlinker", R"(
		4097 4096
		4098 4096
		4099 4096
	)"},
	{"CenterBlinker", R"(
		-1 0
		0 0
		1 0
	)"},
	{"MaxBoundaryBlinker", R"(
		9223372036854775807 9223372036854775807
		9223372036854775806 9223372036854775807
		9223372036854775805 9223372036854775807
	)"},
	{"Glider", R"(
		4097 4096
		4098 4097
		4096 4098
		4097 4098
		4098 4098
	)"} };

int main()
{
	ConwayGameOfLife c;
	c.reset(InputMap["Block"]);
	c.tick(10);
	c.print();
	return 0;
}
