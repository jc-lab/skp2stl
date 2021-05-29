#include <stdio.h>

#include <SketchUpAPI/sketchup.h>
#include <skp2stl/stl_exporter.h>

int main(int argc, char *argv[]) {
  int rc;
  SUResult su_result;
  SUModelRef model = SU_INVALID;
  SUEntitiesRef entities = SU_INVALID;

  if (argc != 4) {
    printf("%s PROJECT_NAME SKP_FILE STL_FILE\n", argv[0]);
    return 1;
  }

  do {
    const char *project_name = argv[1];
    const char *skp_file = argv[2];
    const char *stl_file = argv[3];

    auto exporter = skp2stl::StlExporter::create();

    SUInitialize();

    su_result = SUModelCreateFromFile(&model, skp_file);
    if (su_result != SU_ERROR_NONE) {
      rc = 3;
      break;
    }

    SUModelGetEntities(model, &entities);
    rc = exporter->loadFromEntities(entities);

    SUModelRelease(&model);

    if (rc != 0) {
      return rc;
    }

    std::fstream s;
    s.open(stl_file, std::ios::out);
    exporter->exportToStl(s, project_name);
  } while (false);

  SUTerminate();

  return 0;
}
