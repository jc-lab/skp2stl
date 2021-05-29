/**
 * @file	stl_exporter.cc
 * @author	Joseph Lee <joseph@jc-lab.net>
 * @date	2021/05/29
 * @copyright Copyright (C) 2021 jc-lab.\n
 *            This software may be modified and distributed under the terms
 *            of the Apache License 2.0.  See the LICENSE file for details.
 */

#include <SketchUpAPI/sketchup.h>

#include <memory>
#include <vector>
#include <cstdarg>

#include <skp2stl/stl_exporter.h>

#define pos(a, b) ((a)+((b)*4))

namespace skp2stl {

class Vector3 {
 public:
  double x, y, z;
  Vector3 Cross(const Vector3 &V) {
    Vector3 v;
    v.x = y * V.z - z * V.y;
    v.y = z * V.x - x * V.z;
    v.z = x * V.y - y * V.x;
    return v;
  }
  double Dot(const Vector3 &V) {
    double res;
    res = x * V.x + y * V.y + z * V.z;
    return res;
  }
  Vector3 operator-(const Vector3 V) {
    Vector3 v;
    v.x = x - V.x;
    v.y = y - V.y;
    v.z = z - V.z;
    return v;
  }
};

class cFacet {
 public:
  Vector3 normal, vertex[3];

  Vector3 Middle() {
    Vector3 v;
    v.x = v.y = v.z = 0;
    for (int i = 0; i < 3; i++) {
      v.x += vertex[i].x;
      v.y += vertex[i].y;
      v.z += vertex[i].z;
    }
    v.x /= 3;
    v.y /= 3;
    v.z /= 3;
    return v;
  }
};

class FormattedWriter {
 protected:
  virtual int write(const char *data, size_t length) = 0;

 public:
  void outf(const char *format, ...) {
    char buf[256];
    size_t len;

    std::va_list vl;
        va_start(vl, format);
    len = vsnprintf(buf, sizeof(buf), format, vl);
        va_end(vl);

    write(buf, len);
  }
};

class FstreamWriter : public FormattedWriter {
 private:
  std::fstream &stream_;

 public:
  FstreamWriter(std::fstream &stream) : stream_(stream) {}

 protected:
  int write(const char *data, size_t length) override {
    stream_.write(data, length);
    return 0;
  }
};

class StlExporterImpl : public StlExporter {
 private:
  int triangle_count_;
  int face_count_;
  int entities_count_;
  SUTransformation matrix_i_;
  std::vector<cFacet> facet_arr_;

 public:
  StlExporterImpl() {
  }

  int loadFromEntities(SUEntitiesRef entities) override {
    init();
    return traversalEntities(entities, 0, matrix_i_);
  }

  int exportToStl(std::fstream &stream, const char *project_name) const override {
    FstreamWriter writer(stream);
    return exportToStlImpl(writer, project_name);
  }

 private:
  int exportToStlImpl(FormattedWriter &writer, const char *project_name) const {
    writer.outf("solid %s\n", project_name);

    for (int i = 0; i < facet_arr_.size(); i++) {
      writer.outf(
          "facet normal %lf %lf %lf\nouter loop\n",
          facet_arr_[i].normal.x,
          facet_arr_[i].normal.y,
          facet_arr_[i].normal.z
      );

      for (int j = 0; j < 3; j++) {
        writer.outf("vertex %lf %lf %lf\n",
                    facet_arr_[i].vertex[j].x,
                    facet_arr_[i].vertex[j].y,
                    facet_arr_[i].vertex[j].z);
      }

      writer.outf("endloop\nendfacet\n");
    }

    writer.outf("endsolid %s\n", project_name);

    return 0;
  }

  void init() {
    triangle_count_ = 0;
    face_count_ = 0;
    entities_count_ = 0;
    for (size_t i = 0; i < 16; i++) matrix_i_.values[i] = 0;
    for (size_t i = 0; i < 4; i++) matrix_i_.values[pos(i, i)] = 1;
  }

  void getComponentEntity(SUEntitiesRef entities, int recursive_depth, const SUTransformation &transformation) {
    size_t componentInstanceLen;
    SUEntitiesGetNumInstances(entities, &componentInstanceLen);
    if (componentInstanceLen > 0) {
      std::vector<SUComponentInstanceRef> componentInstanceArr(componentInstanceLen);
      SUEntitiesGetInstances(entities, componentInstanceLen, &componentInstanceArr[0], &componentInstanceLen);
      for (size_t i = 0; i < componentInstanceLen; i++) {

        /*
            Get definition from instance of component
        */
        SUComponentDefinitionRef definitionOfInstance;
        SUComponentInstanceGetDefinition(componentInstanceArr[i], &definitionOfInstance);
        /*
            Get entities from component definition
        */
        SUEntitiesRef entitiesInDefinition;
        SUComponentDefinitionGetEntities(definitionOfInstance, &entitiesInDefinition);

        SUTransformation transforMation2 = {0};
        SUTransformation resTransforMation = {0};
        memset(resTransforMation.values, 0, sizeof(resTransforMation.values));

        SUComponentInstanceGetTransform(componentInstanceArr[i], &transforMation2);

        for (int kk = 0; kk < 4; kk++)
          for (int ii = 0; ii < 4; ii++)
            for (int jj = 0; jj < 4; jj++)
              resTransforMation.values[pos(ii, jj)] +=
                  transformation.values[pos(ii, kk)] * transforMation2.values[pos(kk, jj)];

        traversalEntities(entitiesInDefinition, recursive_depth, resTransforMation);
      }
    }
  }

  void traversalGroupEntity(SUEntitiesRef entities, int recursive_depth, const SUTransformation &transformation) {
    size_t groupLen;
    SUEntitiesGetNumGroups(entities, &groupLen);
    if (groupLen > 0) {
      std::vector<SUGroupRef> groupArr(groupLen);
      SUEntitiesGetGroups(entities, groupLen, &groupArr[0], &groupLen);
      for (size_t i = 0; i < groupLen; i++) {
        SUEntitiesRef entitiesInGroup;
        SUGroupGetEntities(groupArr[i], &entitiesInGroup);

        SUTransformation transforMation2 = {0};
        SUTransformation resTransforMation = {0};
        memset(resTransforMation.values, 0, sizeof(resTransforMation.values));

        SUGroupGetTransform(groupArr[i], &transforMation2);

        for (int kk = 0; kk < 4; kk++)
          for (int ii = 0; ii < 4; ii++)
            for (int jj = 0; jj < 4; jj++)
              resTransforMation.values[pos(ii, jj)] +=
                  transformation.values[pos(ii, kk)] * transforMation2.values[pos(kk, jj)];

        traversalEntities(entitiesInGroup, recursive_depth, resTransforMation);
      }
    }
  }

  void addFaces(SUEntitiesRef entities, const SUTransformation &transformation) {
    int su_result;

    size_t faceLen = 0;
    SUEntitiesGetNumFaces(entities, &faceLen);
    if (faceLen > 0) {
      std::vector<SUFaceRef> faces(faceLen);
      SUEntitiesGetFaces(entities, faceLen, &faces[0], &faceLen);
      for (size_t i = 0; i < faceLen; i++) {
        SUMeshHelperRef face_mesh = SU_INVALID;

        su_result = SUMeshHelperCreate(&face_mesh, faces[i]);

        size_t triangleLen;
        SUMeshHelperGetNumTriangles(face_mesh, &triangleLen);
        std::vector<SUVector3D> normalArr(triangleLen);
        SUMeshHelperGetNormals(face_mesh, triangleLen, &normalArr[0], &triangleLen);

        size_t vertexLen;
        SUMeshHelperGetNumVertices(face_mesh, &vertexLen);
        std::vector<SUPoint3D> vertexArr(vertexLen);
        SUMeshHelperGetVertices(face_mesh, vertexLen, &vertexArr[0], &vertexLen);

        size_t indexLen = triangleLen * 3;
        std::vector<size_t> indexArr(indexLen);
        SUMeshHelperGetVertexIndices(face_mesh, indexLen, &indexArr[0], &indexLen);
        SUMeshHelperRelease(&face_mesh);

        for (size_t i = 0; i < triangleLen; i++) {

          double normal[3] = {normalArr[i].x, normalArr[i].y, normalArr[i].z};
          double normalTrans[3] = {0};
          double vertexTrans[3][3] = {0};
          for (int ii = 0; ii < 3; ii++)
            for (int jj = 0; jj < 3; jj++)
              normalTrans[ii] += normal[jj] * transformation.values[pos(ii, jj)];
          for (size_t j = 0; j < 3; j++) {
            double vertex[3] =
                {vertexArr[indexArr[i * 3 + j]].x, vertexArr[indexArr[i * 3 + j]].y, vertexArr[indexArr[i * 3 + j]].z};
            for (int ii = 0; ii < 3; ii++)
              for (int jj = 0; jj < 3; jj++)
                vertexTrans[j][ii] += vertex[jj] * transformation.values[pos(ii, jj)];
            for (int ii = 0; ii < 3; ii++)
              vertexTrans[j][ii] += transformation.values[pos(ii, 3)];
          }

          double x0 = vertexTrans[1][0] - vertexTrans[0][0];
          double y0 = vertexTrans[1][1] - vertexTrans[0][1];
          double z0 = vertexTrans[1][2] - vertexTrans[0][2];

          double x1 = vertexTrans[2][0] - vertexTrans[0][0];
          double y1 = vertexTrans[2][1] - vertexTrans[0][1];
          double z1 = vertexTrans[2][2] - vertexTrans[0][2];

          double xi = y0 * z1 - z0 * y1;
          double yi = z0 * x1 - x0 * z1;
          double zi = x0 * y1 - y0 * x1;

          double docProduct = xi * normalTrans[0] + yi * normalTrans[1] + zi * normalTrans[2];

          if (docProduct < 0) {
            for (int ii = 0; ii < 3; ii++) {
              std::swap(vertexTrans[1][ii], vertexTrans[2][ii]);
            }
          }

          cFacet aFacet;
          aFacet.normal.x = normalTrans[0];
          aFacet.normal.y = normalTrans[1];
          aFacet.normal.z = normalTrans[2];

          for (int jj = 0; jj < 3; jj++) {
            aFacet.vertex[jj].x = vertexTrans[jj][0];
            aFacet.vertex[jj].y = vertexTrans[jj][1];
            aFacet.vertex[jj].z = vertexTrans[jj][2];
          }

          facet_arr_.push_back(aFacet);

          face_count_++;
        }
        SUMeshHelperRelease(&face_mesh);
      }
    }
  }

  int traversalEntities(SUEntitiesRef entities, int recursive_depth, SUTransformation transformation) {
    if (recursive_depth > 32) {
      return -1;
    }

    recursive_depth += 1;
    addFaces(entities, transformation);
    traversalGroupEntity(entities, recursive_depth, transformation);
    getComponentEntity(entities, recursive_depth, transformation);
    return 0;
  }
};

std::unique_ptr<StlExporter> StlExporter::create() {
  return std::make_unique<StlExporterImpl>();
}

} // namespace skp2stl
