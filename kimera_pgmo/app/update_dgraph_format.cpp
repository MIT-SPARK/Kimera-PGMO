#include <CLI/CLI.hpp>

#include "kimera_pgmo/deformation_graph.h"

struct AppArgs {
  std::filesystem::path input;
  std::filesystem::path output;

  void add_args(CLI::App& app) {
    app.add_option("filepath", input)
        ->check(CLI::ExistingFile)
        ->required()
        ->description("Path to input dgraph file");
    app.add_option("--output", output, "Optional output file");
  }
};

auto main(int argc, char* argv[]) -> int {
  CLI::App app("Node publishing parent_T_child from CSV");
  app.allow_extras();
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_args(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  auto dgraph = kimera_pgmo::DeformationGraph::loadFromFile(args.input);
  if (args.output.empty()) {
    args.output = args.input.replace_extension(".json");
  }

  dgraph->save(args.output);
  return 0;
}
